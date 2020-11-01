// Nucleo LEP
// © 2019-2020  Patrick Lafarguette
//
// LEP file reader/writer for Nucleo-F429ZI
//
// 27/10/2020 STM32Duino core 1.9.0
//            http://github.com/stm32duino/Arduino_Core_STM32/pull/1209
//            http://github.com/stm32duino/Arduino_Core_STM32/pull/1212
//            GFX 1.10.2
//            lvgl 7.7.0
//            Read and write MO5LEP file (1.9.0 API)
//
// 07/10/2020 STM32Duino core 1.9.0
//            GFX 1.10.1
//            TouchScreen 1.1.0
//            lvgl 7.6.0
//            MCUFRIEND_kbv 2.9.9
//            SdFat 1.1.4
//
// 22/05/2020 STM32Duino core 1.9.0
//            GFX 1.10.1
//            TouchScreen 1.1.0
//            lv_arduino 2.0.3
//            MCUFRIEND_kbv 2.9.9
//            SdFat 1.1.4
//
// 20/02/2020 STM32Duino core 1.6.1
//
// 11/11/2019 lv_arduino 2.0.3
//
// 05/10/2019 Read and write MO5LEP file
//
// 25/09/2019 STM32Duino core 1.6.1
//            GFX 1.5.7
//            TouchScreen 1.0.3
//            LittlevGL 2.0.1
//            MCUFRIEND_kbv development
//            SdFat 1.1.0

#include <Adafruit_GFX.h>
#include <lvgl.h>
#include <MCUFRIEND_kbv.h>
#include <SdFat.h>
#include <TouchScreen.h>

#include "LEP.h"
#include "Stack.h"

// TFT calibration
// See TouchScreen_Calibr_native example in MCUFRIEND_kbv library
const int XP = 8, XM = A2, YP = A3, YM = 9;
const int TS_LEFT = 908, TS_RT = 122, TS_TOP = 85, TS_BOT = 905;

MCUFRIEND_kbv tft;
// XP (LCD_RS), XM (LCD_D0) resistance is 345 Ω
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 345);

SdFat fat;
SdFile directory;
SdFile file;

Stack<String*> folders;

// Timers
HardwareTimer* readTimer = NULL;
HardwareTimer* writeTimer = NULL;

#if 0
#elif defined(ARDUINO_NUCLEO_F429ZI)
#define PIN_ACTIVITY LED_BUILTIN
#define PIN_MOTOR PD0
#define PIN_OUTPUT PF7 // TIM6
#define PIN_INPUT PF8 // TIM13, channel 1
#elif defined(ARDUINO_NUCLEO_L476RG)
#define PIN_ACTIVITY LED_BUILTIN
#define PIN_MOTOR PC8
#define PIN_OUTPUT PC6
#endif

#define DEBUG 1 // 1 to enable serial debug messages, 0 to disable
#define CAPTURE 1 // 1 to enable screen captures,  0 to disable

#if DEBUG
#define Debug(...) Serial.print(__VA_ARGS__)
#define Debugln(...) Serial.println(__VA_ARGS__)
#else
#define Debug(...)
#define Debugln(...)
#endif

// Buffer
// Minimum buffer play time is PERIOD * BUFFER_SIZE µs
// Buffer should be loaded in less time
//
// 1024 bytes RAM, 512 bytes buffers
// 25600 µs, 25,600 ms at 50 µs
//  8192 µs,  8,192 ms at 16 µs
//
// 2048 bytes RAM, 1024 bytes buffers
// 51200 µs, 51,200 ms at 50 µs
// 16384 µs, 16,384 ms at 16 µs
#define BUFFER_COUNT 2
#define BUFFER_SIZE 1024

typedef enum states {
	state_idle,
	state_hide,
	state_media,
	state_root,
	state_up,
	state_folder,
	state_file,
	state_play,
	state_record,
	state_read,
	state_write,
	state_eob,
	state_eof,
	state_stop,
	state_settings,
} states_t;

typedef enum actions {
	action_idle,
	action_play,
	action_record,
} actions_t;

typedef enum buttons {
	button_root,
	button_up,
	button_action,
	button_stop,
	button_settings,
} buttons_t;

#define	UI_NONE    0
#define	UI_MOTOR   (1 << 0)
#define	UI_PLAY    (1 << 1)
#define	UI_RECORD  (1 << 2)
#define	UI_CAPTURE (1 << 3)

typedef struct Box {
	uint8_t state;
	const char* message;
} Box;

typedef struct Block {
	uint32_t read; // Read count
	uint32_t write; // Write count
	uint32_t size; // Size
	uint32_t available;
	uint32_t count;
} Block;

typedef struct Worker {
	// State
	uint8_t state;
	uint8_t action;
	uint8_t ui;
	// Box
	Box box;
	// Folders
	char* filename;
	// Buffers
	int8_t buffers[BUFFER_COUNT][BUFFER_SIZE];
	int8_t buffer;
	uint16_t index;
	// File
	bool eof;
	uint32_t size;
	// Block
	Block block;
	// Play or record
	bool motor;
	bool remote;
	uint8_t active; // Active LOW or HIGH
	uint8_t output; // Initial output LOW or HIGH
	uint8_t input;
	uint8_t period; // Default 50 µs
	uint32_t counter;
	uint32_t overflow;
	uint32_t overcapture;
	unsigned long start;
	unsigned long stop;
} Worker;

Worker worker;

// User interface
#define LV_SYMBOL_RECORD "\xef\x84\x91"

static lv_disp_buf_t lvgl_disp_buf;
static lv_color_t lvgl_buf[LV_HOR_RES_MAX * 10];

lv_obj_t* lvgl_files = NULL;
lv_obj_t* lvgl_toolbar = NULL;
lv_obj_t* lvgl_box = NULL;
lv_obj_t* lvgl_play = NULL;
lv_obj_t* lvgl_bar = NULL;
lv_obj_t* lvgl_filename = NULL;
lv_obj_t* lvgl_size = NULL;
lv_obj_t* lvgl_motor = NULL;

#define TOOLBAR_NONE   0
#define TOOLBAR_PLAY   (1 << 0)
#define TOOLBAR_ACTION (1 << 1)
#define TOOLBAR_STOP   (1 << 2)

//////////
// LVGL //
//////////

// Log to serial
#if LV_USE_LOG != 0
void lvgl_print(lv_log_level_t level, const char* file, uint32_t line, const char* function, const char* description) {
	UNUSED(level);
	UNUSED(function);
	Debug(file);
	Debug("@");
	Debug(line);
	Debug("->");
	Debugln(description);
	Serial.flush();
	delay(100);
}
#endif

// Display
void lvgl_disp_flush(lv_disp_drv_t* display, const lv_area_t* area, lv_color_t* color_p) {
	tft.setAddrWindow(area->x1, area->y1, area->x2, area->y2);
	tft.pushColors((uint16_t*)color_p, (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1), true);
	lv_disp_flush_ready(display);
}

// Input
#define TS_MIN 10
#define TS_MAX 1000

bool lvgl_input_read(lv_indev_drv_t* driver, lv_indev_data_t* data) {
	UNUSED(driver);
	static lv_indev_data_t last = { { 0, 0 }, 0, 0, 0, LV_INDEV_STATE_REL };
	TSPoint point = ts.getPoint();
	pinMode(YP, OUTPUT);
	pinMode(XM, OUTPUT);
	digitalWrite(YP, HIGH);
	digitalWrite(XM, HIGH);
	if (point.z == 0) {
		// Invalid, no change
		data->point = last.point;
		data->state = last.state;
	} else {
		// Valid, change
		// TODO adapt code to your display
		data->point.x = map(point.x, TS_LEFT, TS_RT, 0, tft.width());;
		data->point.y = map(point.y, TS_TOP, TS_BOT, 0, tft.height());
		if ((point.z > TS_MIN) && (point.z < TS_MAX)) {
			data->state = LV_INDEV_STATE_PR;
		} else {
			data->point = last.point;
			data->state = LV_INDEV_STATE_REL;
		}
	}
	last.point = data->point;
	last.state = data->state;
	return false;
}

// Initialize
void lvgl_init(void) {
#if LV_USE_LOG != 0
	lv_log_register_print_cb(lvgl_print);
#endif
	// Initialize the library
	lv_init();
	lv_disp_buf_init(&lvgl_disp_buf, lvgl_buf, NULL, LV_HOR_RES_MAX * 10);
	// Initialize the display driver
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = LV_HOR_RES_MAX;
	disp_drv.ver_res = LV_VER_RES_MAX;
	disp_drv.flush_cb = lvgl_disp_flush;
	disp_drv.buffer = &lvgl_disp_buf;
	lv_disp_drv_register(&disp_drv);
	// Initialize the touch pad driver
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = lvgl_input_read;
	lv_indev_drv_register(&indev_drv);
}

// Setup the UI elements
void lvgl_setup(void) {
	// Toolbar zone
	{
		lvgl_toolbar = lv_btnmatrix_create(lv_scr_act(), NULL);
		lv_obj_set_event_cb(lvgl_toolbar, lvgl_button_action);
		lv_obj_set_size(lvgl_toolbar, LV_HOR_RES, 40);
		lv_obj_align(lvgl_toolbar, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);
	}
	// Files zone
	{
		// Create the list
		lvgl_files = lv_list_create(lv_scr_act(), NULL);
		lv_obj_set_size(lvgl_files, tft.width(), tft.height() - 40);
		lv_obj_align(lvgl_files, lvgl_toolbar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
		lv_list_set_scrollbar_mode(lvgl_files, LV_SCRLBAR_MODE_AUTO);
	}
	// Play zone
	{
		// Create the container
		lvgl_play = lv_cont_create(lv_scr_act(), NULL);
		lv_obj_set_size(lvgl_play, tft.width(), tft.height() - 40);
		lv_obj_align(lvgl_play, lvgl_toolbar, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
		lv_obj_set_hidden(lvgl_play, true);
		lvgl_filename = lv_label_create(lvgl_play, NULL);
		lv_label_set_long_mode(lvgl_filename, LV_LABEL_LONG_SROLL);
		lv_obj_set_width(lvgl_filename, tft.width() - 20);
		lv_obj_align(lvgl_filename, NULL, LV_ALIGN_IN_TOP_LEFT, 10, 10);
		lvgl_size = lv_label_create(lvgl_play, NULL);
		lv_label_set_long_mode(lvgl_size, LV_LABEL_LONG_SROLL);
		lv_obj_set_width(lvgl_size, tft.width() - 20);
		lv_obj_align(lvgl_size, lvgl_filename, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
		lvgl_motor = lv_checkbox_create(lvgl_play, NULL);
		lv_checkbox_set_text(lvgl_motor, "Moteur");
		lv_obj_align(lvgl_motor, lvgl_size, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 10);
		// Create a bar
		lvgl_bar = lv_bar_create(lvgl_play, NULL);
		lv_obj_set_size(lvgl_bar, tft.width() - 20, 30);
		lv_obj_align(lvgl_bar, NULL, LV_ALIGN_IN_BOTTOM_MID, 0, -10);
	}
}

// Update the UI from application state
void lvgl_update(const uint16_t state) {
	static const char* lvgl_buttons[] = {
			LV_SYMBOL_HOME,
			LV_SYMBOL_UP,
			LV_SYMBOL_RECORD,
			LV_SYMBOL_STOP,
			LV_SYMBOL_SETTINGS,
			""
	};
	lv_btnmatrix_ctrl_t controls[] = { 0, 0, 0, 0, 0 };
	// Display play or record
	if (worker.action == action_play) {
		lvgl_buttons[button_action] = LV_SYMBOL_PLAY;
	} else {
		lvgl_buttons[button_action] = LV_SYMBOL_RECORD;
	}
	// Disable root and up buttons
	if ((folders.count() < 2) | (state & TOOLBAR_STOP)) {
		controls[button_root] = controls[button_up] = LV_BTNMATRIX_CTRL_DISABLED;
	}
	// Disable action button, play or record
	if (~state & TOOLBAR_ACTION) {
		controls[button_action] = LV_BTNMATRIX_CTRL_DISABLED;
	}
	// Disable stop button
	if (~state & TOOLBAR_STOP) {
		controls[button_stop] = LV_BTNMATRIX_CTRL_DISABLED;
	}
	lv_btnmatrix_set_map(lvgl_toolbar, lvgl_buttons);
	lv_btnmatrix_set_ctrl_map(lvgl_toolbar, controls);
}

// Display a message box
void lvgl_box_display() {
	if (lvgl_box == NULL) {
		// Set colors to styles
		switch (worker.box.state) {
		case state_media:
			// TODO Alarm, red
			break;
		case state_eob:
		case state_eof:
		case state_settings:
			// TODO Information, green
			break;
		}
		// Create the message box
		static const char* buttons[] = { "Fermer", "" };
		lvgl_box = lv_msgbox_create(lv_scr_act(), NULL);
		lv_msgbox_set_text(lvgl_box, worker.box.message);
		lv_msgbox_add_btns(lvgl_box, buttons);
		lv_obj_set_event_cb(lvgl_box, lvgl_box_action);
		lv_obj_set_width(lvgl_box, 200);
		lv_obj_align(lvgl_box, NULL, LV_ALIGN_CENTER, 0, 0);
		// TODO Apply styles
	}
}

// Handle box action
void lvgl_box_action(lv_obj_t* box, lv_event_t event) {
	UNUSED(box);
	if (event == LV_EVENT_VALUE_CHANGED) {
		switch (worker.box.state) {
		case state_media:
			if (fat.begin()) {
				worker.state = state_hide;
				worker.box.state = state_root;
			}
			break;
		case state_eob:
		case state_eof:
		case state_settings:
			worker.state = state_hide;
			worker.box.state = state_idle;
			break;
		}
	}
}

// Compare function to sort folders and files stacks
bool compare(String*& a, String*& b) {
	return *a > *b;
}

#define FILENAME_SIZE 256

// Load list with folders and files names
void lvgl_files_load(void) {
	char buffer[FILENAME_SIZE];
	Stack<String*> folders;
	Stack<String*> files;
	// Clean the list
	lv_list_clean(lvgl_files);
	// Build the list
	fat.vwd()->rewind();
	fat.vwd()->getName(buffer, FILENAME_SIZE);
	// Loop files
	while (file.openNext(fat.vwd(), O_RDONLY)) {
		file.getName(buffer, FILENAME_SIZE);
		if (!file.isHidden()) {
			if (file.isDir()) {
				folders.push(new String(buffer));
			} else {
				files.push(new String(buffer));
			}
		}
		file.close();
	}
	// Sort, folders then files, alphabetical order
	folders.sort(compare);
	files.sort(compare);
	// Load
	for (unsigned int index = 0; index < folders.count(); ++index) {
		lv_obj_t* button = lv_list_add_btn(lvgl_files, LV_SYMBOL_DIRECTORY, folders[index]->c_str());
		lv_obj_set_event_cb(button, lvgl_folder_action);
	}
	for (unsigned int index = 0; index < files.count(); ++index) {
		lv_obj_t* button = lv_list_add_btn(lvgl_files, LV_SYMBOL_FILE, files[index]->c_str());
		lv_obj_set_event_cb(button, lvgl_file_action);
	}
}

// Handle folder action, open folder
void lvgl_folder_action(lv_obj_t* button, lv_event_t event) {
	if (event == LV_EVENT_CLICKED) {
		worker.filename = (char*)lv_list_get_btn_text(button);
		if (file.open(worker.filename, O_RDONLY)) {
			worker.state = state_folder;
			file.close();
		}
	}
}

// Handle file action, open file
void lvgl_file_action(lv_obj_t* button, lv_event_t event) {
	if (event == LV_EVENT_CLICKED) {
		worker.filename = (char*)lv_list_get_btn_text(button);
		if (file.open(worker.filename, O_RDONLY)) {
			worker.state = state_file;
		}
	}
}

// Handle buttons action
void lvgl_button_action(lv_obj_t* toolbar, lv_event_t event) {
	uint16_t button = lv_btnmatrix_get_active_btn(toolbar);
	if (event == LV_EVENT_CLICKED) {
		if (!lv_btnmatrix_get_btn_ctrl(toolbar, button, LV_BTNMATRIX_CTRL_DISABLED)) {
			switch (button) {
			case button_root:
				worker.state = state_root;
				Debugln("root button clicked");
				break;
			case button_up:
				worker.state = state_up;
				Debugln("up button clicked");
				break;
			case button_action: // Record or play
				if (worker.action == action_play) {
					worker.state = state_play;
					Debugln("play button clicked");
				} else {
					worker.state = state_record;
					Debugln("record button clicked");
				}
				break;
			case button_stop:
				worker.state = state_stop;
				Debugln("stop button clicked");
				break;
			case button_settings:
				worker.state = state_settings;
				Debugln("settings button clicked");
				break;
			default:
				break;
			}
		}
	}
}

///////////////////////
// Interrupt handler //
///////////////////////

// Tick UI every millisecond
void HAL_SYSTICK_Callback(void) {
	lv_tick_inc(1);
}

// Timer output sample to computer
void readIRQHandler(void) {
	// One pulse
	// TODO TIM_SR_UIF not cleared by HardwareTimer class
	TIM6->SR = ~TIM_SR_UIF;
	lep_output();
}

// Timer capture input sample from computer
void writeIRQHandler(void) {
	// Edge detection
	// TODO assume TIM_SR_CC1IF cleared by HardwareTimer class
	TIM13->CNT = 0;
	lep_input();
}

void rolloverIRQHandler(void) {
	// TODO assume TIM_SR_CC1OF not cleared by HardwareTimer class
	// Overcapture
	if (TIM13->SR & TIM_SR_CC1OF) {
		TIM13->SR = ~TIM_SR_CC1OF;
		worker.overcapture++;
	}
	// TODO assume TIM_SR_UIF cleared by HardwareTimer class
	worker.overflow++;
}

// Motor
void motorIRQHandler() {
	worker.motor = digitalRead(PIN_MOTOR) == worker.active;
	if (worker.action == action_record) {
		if (worker.motor) {
			worker.input = digitalRead(PIN_INPUT);
			// Counter reset
			TIM13->CNT = 0;
			// Enable interrupts
			TIM13->DIER |= TIM_DIER_CC1IE | TIM_DIER_UIE;
			// Enable timer
			TIM13->CR1 |= TIM_CR1_CEN;
		} else {
			// Disable timer
			TIM13->CR1 &= ~TIM_CR1_CEN;
			// Disable interrupts
			TIM13->DIER &= ~(TIM_DIER_CC1IE | TIM_DIER_UIE);
			// Input
			lep_input();
		}
	} else {
		if (worker.motor) {
			// TODO check something to be done?
		} else {
			// TODO Thomson LEP presence is output high
			// TODO not sure if needed
			digitalWrite(PIN_OUTPUT, HIGH);
		}
	}
	worker.ui |= UI_MOTOR;
}

// User button
void userButtonIRQHandler() {
	worker.ui |= UI_CAPTURE;
}

/////////////
// SD card //
/////////////

#if CAPTURE

void sd_screen_capture() {
	char filename[14];
	unsigned int index = 0;
	do  {
		sprintf(filename, "%08d.data", ++index);
	} while (fat.exists(filename));
	Serial.print("Save ");
	Serial.print(filename);
	uint16_t* pixels = (uint16_t*)malloc(sizeof(uint16_t) * tft.width());
	if (pixels) {
		SdFile file;
		if (file.open(filename, O_CREAT | O_RDWR)) {
			for (int y = 0; y < tft.height(); ++y) {
				tft.readGRAM(0, y, pixels, tft.width(), 1);
				file.write(pixels, sizeof(uint16_t) * tft.width());
			}
		}
		file.close();
	} else {
		Serial.println(" failed");
	}
	free(pixels);
	Serial.println(" done");
}

#endif

/////////
// LEP //
/////////

// Initialize IO pins
void lep_init() {
	// Activity
	pinMode(PIN_ACTIVITY, OUTPUT);
	digitalWrite(PIN_ACTIVITY, LOW);
	// Motor
	pinMode(PIN_MOTOR, INPUT_PULLUP);
	// Output
	pinMode(PIN_OUTPUT, OUTPUT);
	// TODO change default OUTPUT
	digitalWrite(PIN_OUTPUT, HIGH);
	// Input
	pinMode(PIN_INPUT, INPUT);
	pin_function(digitalPinToPinName(PIN_INPUT), STM_PIN_DATA_EXT(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF9_TIM13, 1, 0));
}

// Open file to play
void lep_file() {
	lep_header header;
	lep_block block;
	// Worker
	worker.eof = false;
	worker.size = file.fileSize();
	// Header
	file.read(&header, sizeof(header));
	if (header.magic == LEP_MAGIC) {
		// LEP structured file
		worker.period = header.period;
		switch (header.brand) {
		case LEP_EXELVISION:
		case LEP_MATRA:
		case LEP_MATTEL_ELECTRONICS:
			worker.remote = false;
			break;
		default:
			worker.remote = true;
			worker.active = LOW;
			break;
		}
		// Read block
		file.read(&block, sizeof(block));
		worker.block.size = worker.block.available = worker.block.count = block.size;
	} else {
		// LEP raw file
		worker.period = LEP_PERIOD;
		worker.remote = true;
		worker.active = LOW;
		// Default block
		file.rewind();
		worker.block.size = worker.block.available = worker.block.count = worker.size;
	}
	worker.block.read = 0;
	// Action
	worker.action = action_play;
	// Debug
	Debugln("File opened ");
	Debug(worker.size);
	Debugln(" bytes");
}

// Create file to record
void lep_create() {
	static char filename[13];
	unsigned int index = 0;
	lep_header header;
	lep_block block;
	worker.filename = NULL;
	do  {
		sprintf(filename, "%08d.lep", ++index);
	} while (fat.exists(filename));
	worker.filename = filename;
	worker.block.write = 0;
	if (file.open(worker.filename, O_CREAT | O_RDWR)) {
		// Header
		header.magic = LEP_MAGIC;
		header.version = LEP_VERSION;
		header.period =  worker.period;
		header.blocks = 1;
		file.write(&header, sizeof(header));
		// Block
		file.write(&block, sizeof(block));
	}
	worker.block.size = worker.block.count = 0;
}

// Motor
void lep_motor() {
	// Debug
	Debug("Motor ");
	Debugln(worker.motor ? "on" : "off");
}

// Play
void lep_play() {
	// Worker
	worker.start = millis();
	worker.index = 0;
	worker.output = HIGH;
	// Read buffers
	worker.buffer = 1; // Buffer #0
	lep_read();
	worker.buffer = 0; // Buffer #1
	lep_read();
	// Motor
	if (worker.remote) {
		worker.motor = digitalRead(PIN_MOTOR) == worker.active;
		attachInterrupt(digitalPinToInterrupt(PIN_MOTOR), motorIRQHandler, CHANGE);
	} else {
		worker.motor = true;
	}
	worker.ui |= UI_MOTOR;
	// Disable all interrupts
	noInterrupts();
	// Initialize Timer 6
	// - enable TIM6 clock
	// - set prescaler for 1 µs resolution
	// - one pulse mode
	// - force update
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	TIM6->PSC = (uint32_t)(readTimer->getTimerClkFreq() / (1000000)) - 1;
	TIM6->CR1 = TIM_CR1_OPM;
	// Clear the update flag
	TIM6->SR = ~TIM_SR_UIF;
	// Enable interrupt on update event
	TIM6->DIER |= TIM_DIER_UIE;
	 // Enable TIM6 IRQ handler
	HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_IRQn);
	// Output
	lep_output();
	// Enable all interrupts
	interrupts();
}

// Record
void lep_record() {
	// Worker
	worker.start = millis();
	worker.buffer = 0;
	worker.index = 0;
	worker.remote = true;
	worker.period = LEP_PERIOD;
	worker.overflow = 0;
	worker.overcapture = 0;
	// Create file
	lep_create();
	// Clear buffers
	memset(worker.buffers[0], 0, BUFFER_SIZE);
	memset(worker.buffers[1], 0, BUFFER_SIZE);
	// Motor
	if (worker.remote) {
		worker.motor = digitalRead(PIN_MOTOR) == worker.active;
		attachInterrupt(digitalPinToInterrupt(PIN_MOTOR), motorIRQHandler, CHANGE);
	} else {
		worker.motor = true;
//		worker.input = digitalRead(PIN_INPUT);
	}
	worker.ui |= UI_MOTOR | UI_RECORD;
	// Disable all interrupts
	noInterrupts();
	// Initialize Timer 13
	// - enable TIM13 clock
	// - set prescaler for 1 µs resolution
	// - capture mode
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
	TIM13->PSC = (uint32_t)(writeTimer->getTimerClkFreq() / (1000000)) - 1;
	// Reset counter
	TIM13->CNT = 0;
	// Configured as input
	TIM13->CCMR1 |= TIM_CCMR1_CC1S_0;
	// Both edges, capture enabled
	TIM13->CCER |= TIM_CCER_CC1NP | TIM_CCER_CC1P | TIM_CCER_CC1E;
	// Clear the update and channel 1 flags
	TIM13->SR = ~(TIM_SR_CC1OF | TIM_SR_CC1IF | TIM_SR_UIF);
	// Enable TIM13 IRQ handler
	HAL_NVIC_SetPriority(TIM13_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM13_IRQn);
//	if (worker.motor) {
//		writeTimer->resume();
//	}
	// Enable all interrupts
	interrupts();
}

// Output LEP sample to computer
void lep_output() {
	if (worker.motor) {
		// Motor on
		if (worker.block.count--)	{
			int8_t byte = worker.buffers[worker.buffer][worker.index++];
			if (byte == 0) {
				// 127 * 50 µs
				TIM6->ARR = 127 * worker.period - 1;
				TIM6->CR1 |= TIM_CR1_CEN;
			} else {
				// |byte| * 50 µs
				TIM6->ARR = abs(byte) * worker.period - 1;
				TIM6->CR1 |= TIM_CR1_CEN;
				worker.output = byte > 0 ? HIGH : LOW;
			}
			digitalWrite(PIN_OUTPUT, worker.output);
			// Buffers
			if (worker.index == BUFFER_SIZE) {
				worker.buffer ^= 1;
				worker.index = 0;
				worker.state = state_read;
			}
		} else {
			// Worker
			worker.stop = millis();
			worker.state = worker.eof ? state_eof : state_eob;
		}
	} else {
		// Motor off, delay 50 µs
		// TODO restart timer on motor change
		TIM6->ARR = worker.period - 1;
		TIM6->CR1 |= TIM_CR1_CEN;
	}
}

// Input LEP sample from computer
void lep_input() {
	uint32_t counter;
	uint16_t zeros;
	int8_t byte;
	counter = (TIM13->CCR1 + (worker.overflow << 16)) / worker.period;
	worker.overflow = 0;
	zeros = counter / 127;
	byte = counter % 127;
	if (byte == 0) {
		byte++;
	}
	// Toggle
	worker.input ^= HIGH;
	if (worker.input == LOW) {
		 byte = -byte;
	}
	// Byte
	worker.buffers[worker.buffer][worker.index++] = byte;
	// Zeros
	worker.block.count += (zeros + 1);
	worker.index += zeros;
	// Buffers
	if (worker.index >= BUFFER_SIZE) {
		worker.buffer ^= 1;
		worker.index -= BUFFER_SIZE;
		worker.state = state_write;
	}
	worker.ui |= UI_RECORD;
}

// Read buffer from file
void lep_read() {
#ifdef DEBUG
	unsigned long start = micros();
#endif
	unsigned int count = MIN(worker.block.available, BUFFER_SIZE);
	int read = file.read(worker.buffers[worker.buffer ^ 1], count);
	worker.eof = !file.available();
	worker.block.read++;
	worker.block.available -= read;
	// Toggle LED
	digitalWrite(PIN_ACTIVITY, digitalRead(PIN_ACTIVITY) ^ 1);
	// Debug
	Debug(worker.block.read);
	Debug(" read ");
	Debug(read);
	Debug(" bytes in buffer ");
	Debug(worker.buffer ^ 1);
	Debug(" count ");
	Debug(worker.block.count);
	Debug(" time ");
	Debug(micros() - start);
	Debugln(" µs");
	if (worker.eof) {
		Debugln("EOF ");
	}
}

// Write buffer to file
void lep_write() {
#ifdef DEBUG
	unsigned long start = micros();
#endif
	int write = file.write(worker.buffers[worker.buffer ^ 1], BUFFER_SIZE);
	memset(worker.buffers[worker.buffer ^ 1], 0, BUFFER_SIZE);
	worker.block.write++;
	worker.block.size += write;
	// Toggle LED
	digitalWrite(PIN_ACTIVITY, digitalRead(PIN_ACTIVITY) ^ 1);
	// Debug
	Debug(worker.block.write);
	Debug(" write ");
	Debug(write);
	Debug(" bytes in buffer ");
	Debug(worker.buffer ^ 1);
	Debug(" count ");
	Debug(worker.block.count);
	Debug(" time ");
	Debug(micros() - start);
	Debugln(" µs");
}

// End of block play
void lep_eob() {
	lep_block block;
	// Motor
	if (worker.remote) {
		worker.motor = digitalRead(PIN_MOTOR) == worker.active;
		detachInterrupt(digitalPinToInterrupt(PIN_MOTOR));
	} else {
		worker.motor = false;
	}
	worker.ui |= UI_MOTOR;
	// Read block
	file.read(&block, sizeof(block));
	worker.block.size = worker.block.available = worker.block.count = block.size;
	worker.block.read = 0;
	// Debug
	Debug("Total time ");
	Debug(worker.stop - worker.start);
	Debugln(" ms");
}

// End of file play
void lep_eof() {
	// Debug
	Debug("Total time ");
	Debug(worker.stop - worker.start);
	Debugln(" ms");
}

// Stop play or record
void lep_stop() {
	lep_header header;
	lep_block block;
	// TODO Thomson LEP presence is output high
	digitalWrite(PIN_OUTPUT, HIGH);
	// Motor
	if (worker.remote) {
		worker.motor = digitalRead(PIN_MOTOR) == worker.active;
		detachInterrupt(digitalPinToInterrupt(PIN_MOTOR));
	} else {
		worker.motor = false;
	}
	// Action
	switch (worker.action) {
	case action_play:
		// Disable timer
		TIM6->CR1 &= ~TIM_CR1_CEN;
		// Close file
		file.close();
		break;
	case action_record:
		// Disable timer
		TIM13->CR1 &= ~TIM_CR1_CEN;
		// Buffer
		if (worker.index > 1) {
			--worker.index; // Remove unused last byte
#ifdef DEBUG
			unsigned long start = micros();
#endif
			int write = file.write(worker.buffers[worker.buffer], worker.index);
			worker.block.write++;
			worker.block.size += write;
			// Debug
			Debug(worker.block.write);
			Debug(" write ");
			Debug(write);
			Debug(" bytes in buffer ");
			Debug(worker.buffer);
			Debug(" count ");
			Debug(worker.block.count);
			Debug(" time ");
			Debug(micros() - start);
			Debugln(" µs");
		}
		// Block
		block.size = worker.block.size;
		// Write
		file.rewind();
		file.read(&header, sizeof(header));
		file.rewind();
		file.write(&header, sizeof(header));
		file.write(&block, sizeof(block));
		file.close();
		// UI
		lvgl_files_load();
		break;
	}
	digitalWrite(PIN_ACTIVITY, LOW);
	// Action
	worker.action = action_record;
}

/////////////
// Arduino //
/////////////

void setup() {
	Serial.begin(115200);
	Serial.println(F("Nucleo LEP 1.0.0"));
	Serial.println(F(__DATE__));
	Serial.println(F(__TIME__));
	Serial.print(F("CPU frequency "));
	Serial.print(0.000001 * F_CPU);
#if defined(__OPTIMIZE_SIZE__)
	Serial.println(F("MHz -Os"));
#else
	Serial.println(F("MHz"));
#endif
	Serial.println(F("Setup"));
	// Timer
	readTimer = new HardwareTimer(TIM6);
	readTimer->attachInterrupt(readIRQHandler);
	writeTimer = new HardwareTimer(TIM13);
	writeTimer->attachInterrupt(1, writeIRQHandler);
	writeTimer->attachInterrupt(rolloverIRQHandler);
	Serial.println(F("Timer"));
	// User button
	attachInterrupt(USER_BTN, userButtonIRQHandler, LOW);
	// TFT
	tft.reset();
	uint16_t identifier = tft.readID();
	tft.begin(identifier);
	Serial.print(F("TFT 0x"));
	Serial.println(identifier, HEX);
	// LVGL
	lvgl_init();
	lvgl_setup();
	lvgl_update(TOOLBAR_NONE);
	// LEP
	lep_init();
	// SD card
	worker.state = fat.begin() ? state_root : state_media;
}

void loop() {
	// User interface
	if (worker.ui) {
		if (worker.ui & UI_MOTOR) {
			lep_motor();
 			lv_checkbox_set_checked(lvgl_motor, worker.motor);
		}
		if (worker.ui & UI_PLAY) {
			// TODO UI play update
		}
		if (worker.ui & UI_RECORD) {
			String format(worker.block.count);
			format += " octets";
			lv_label_set_text(lvgl_size, format.c_str());
		}
		if (worker.ui & UI_CAPTURE) {
			sd_screen_capture();
		}
		worker.ui = UI_NONE;
	}
	lv_task_handler();
	// Worker
	switch (worker.state) {
		case state_idle: // Nothing
			break;
		case state_hide: // Hide the message box
			// UI
			lv_obj_del(lvgl_box);
			lvgl_box = NULL;
			// State
			worker.state = worker.box.state;
			break;
		case state_media: // Ask user to insert a µSD Card
			// UI
			lvgl_update(TOOLBAR_NONE);
			worker.box.state = worker.state;
			worker.box.message = "Insérez une carte\nmicro SD.";
			lvgl_box_display();
			// State
			worker.state = state_idle;
			// Action
			worker.action = action_idle;
			break;
		case state_root: // Move to root and browse directory
			folders.clear();
			folders.push(new String("/"));
			fat.chdir();
			// Action
			worker.action = action_record;
			// UI
			lvgl_files_load();
			lvgl_update(TOOLBAR_ACTION);
			// State
			worker.state = state_idle;
			break;
		case state_up: // Open parent folder and browse directory
			folders.pop();
			fat.chdir();
			for (unsigned int index = 0; index < folders.count(); ++index) {
				fat.chdir(folders[index]->c_str());
			}
			// UI
			lvgl_files_load();
			lvgl_update(TOOLBAR_ACTION);
			// State
			worker.state = state_idle;
			break;
		case state_folder: // Open a folder and browse directory
			folders.push(new String(worker.filename));
			fat.chdir(worker.filename, true);
			// UI
			lvgl_files_load();
			lvgl_update(TOOLBAR_ACTION);
			// State
			worker.state = state_idle;
			break;
		case state_file: // Open file
			lep_file();
			// UI
			lvgl_update(TOOLBAR_PLAY | TOOLBAR_ACTION | TOOLBAR_STOP);
			lv_label_set_text(lvgl_filename, worker.filename);
			{
				String format(worker.size);
				format += " octets";
				lv_label_set_text(lvgl_size, format.c_str());
			}
			lv_checkbox_set_checked(lvgl_motor, worker.motor);
			lv_bar_set_value(lvgl_bar, 0, LV_ANIM_OFF);
			lv_obj_set_hidden(lvgl_files, true);
			lv_obj_set_hidden(lvgl_bar, false);
			lv_obj_set_hidden(lvgl_play, false);
			// State
			worker.state = state_idle;
			break;
		case state_play: // Play file
			lep_play();
			// UI
			lvgl_update(TOOLBAR_PLAY | TOOLBAR_STOP);
			lv_bar_set_range(lvgl_bar, 0, worker.block.size / BUFFER_SIZE);
			lv_bar_set_value(lvgl_bar, 0, LV_ANIM_ON);
			// State
			worker.state = state_idle;
			break;
		case state_record: // Record file
			lep_record();
			// UI
			lvgl_update(TOOLBAR_STOP);
			lv_label_set_text(lvgl_filename, worker.filename);
			lv_label_set_text(lvgl_size, "0 octets");
			lv_checkbox_set_checked(lvgl_motor, worker.motor);
			lv_bar_set_value(lvgl_bar, 0, LV_ANIM_OFF);
			lv_obj_set_hidden(lvgl_files, true);
			lv_obj_set_hidden(lvgl_bar, true);
			lv_obj_set_hidden(lvgl_play, false);
			// State
			worker.state = state_idle;
			break;
		case state_read: // Read buffer from file
			lep_read();
			// UI
			lv_bar_set_value(lvgl_bar, worker.block.read, LV_ANIM_ON);
			// State
			worker.state = state_idle;
			break;
		case state_write: // Write buffer to file
			lep_write();
			// State
			worker.state = state_idle;
			break;
		case state_eob: // End of block
			lep_eob();
			// UI
			lvgl_update(TOOLBAR_PLAY | TOOLBAR_ACTION | TOOLBAR_STOP);
			worker.box.state = worker.state;
			worker.box.message = "La lecture du bloc\nest terminée.";
			lvgl_box_display();
			// State
			worker.state = state_idle;
			break;
		case state_eof: // End of file
			lep_eof();
			// UI
			worker.box.state = worker.state;
			worker.box.message = "La lecture du fichier\nest terminée.";
			lvgl_box_display();
			// State
			worker.state = state_idle;
			break;
		case state_stop: // Stop file operation and display directory
			lep_stop();
			// UI
			lvgl_update(TOOLBAR_ACTION);
			lv_checkbox_set_checked(lvgl_motor, worker.motor);
			lv_bar_set_value(lvgl_bar, 0, LV_ANIM_OFF);
			lv_obj_set_hidden(lvgl_files, false);
			lv_obj_set_hidden(lvgl_play, true);
			// State
			worker.state = state_idle;
			break;
		case state_settings: // Display settings
			// UI
			worker.box.state = worker.state;
			worker.box.message = "Nucleo LEP\n1.0.0\nCore 1.9.0\n" __DATE__ "\n" __TIME__;
			lvgl_box_display();
			// State
			worker.state = state_idle;
			break;
		default:
			break;
	}
}

#define F_CPU 1000000 // Set the CPU frequency to 1 MHz
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>

//[-------------Global---------------]
// DS18B20 commands
#define DS18B20_CMD_CONVERTTEMP 0x44 // Command to start temperature conversion
#define DS18B20_CMD_RSCRATCHPAD 0xBE // Command to read scratchpad
#define DS18B20_CMD_SKIPROM 0xCC // Command to skip ROM

#define DS18B20_PORT PORTA // Port for DS18B20
#define DS18B20_DDR DDRA // Data direction register for DS18B20
#define DS18B20_PIN PA0 // Pin for DS18B20
#define DS18B20_PINR PINA // Pin input register for DS18B20

#define RS 7 // Register Select pin for LCD
#define E 6  // Enable pin for LCD

//[-------------Prototyp---------------]
void lcd_init(void); // Initializes the LCD
void lcd_cmd(uint8_t cmd); // Sends a command to the LCD
void lcd_data(char data); // Sends data (a character) to the LCD
void lcd_print(char *str); // Prints a string on the LCD
void lcd_set_cursor(uint8_t row, uint8_t col); // Sets the cursor position on the LCD
void display_test(void); // Displays test messages on the LCD
void lcd_test(void); // Another test function for the LCD
void display_temperatures_test(int temperature, int unused); // Displays temperature from DS18B20 on the LCD
void int_to_string_fixed(char* buffer, int value, uint8_t width); // Converts an integer to a fixed-width string

uint8_t ds18b20_reset(void); // Resets the DS18B20 sensor
void ds18b20_write_bit(uint8_t bit); // Writes a bit to the DS18B20 sensor
uint8_t ds18b20_read_bit(void); // Reads a bit from the DS18B20 sensor
void ds18b20_write_byte(uint8_t byte); // Writes a byte to the DS18B20 sensor
uint8_t ds18b20_read_byte(void); // Reads a byte from the DS18B20 sensor
int ds18b20_get_temp(void); // Gets the temperature from the DS18B20 sensor


int main(void)
{
	lcd_init(); 
	
	while (1)
	{
		int temperature = ds18b20_get_temp(); 
		display_temperatures_test(temperature, 0); 
		_delay_ms(1000); 
	}
}


//[------------Function-----------]
void lcd_init(void){
	DDRD = 0xFF; // Set PORTD as output for LCD data
	DDRC |= ((1 << E)|(1 << RS)); // Set RS and E as output
	_delay_ms(100); // Delay for LCD power on
	lcd_cmd(0x30); // Initialize LCD in 8-bit mode
	lcd_cmd(0x30); // Repeat initialization
	lcd_cmd(0x30); // Repeat initialization
	lcd_cmd(0x38); // Function set: 8-bit, 2 line, 5x7 dots
	lcd_cmd(0x0E); // Display on, cursor on
	lcd_cmd(0x06); // Entry mode, increment cursor
	lcd_cmd(0x01); // Clear display
}

void lcd_cmd(uint8_t cmd){
	PORTC &= ~(1 << RS); // RS = 0 for command
	PORTD = cmd; // Put command on data bus
	PORTC |= (1 << E); // Enable pulse
	_delay_us(5);
	PORTC &= ~(1 << E); // Disable pulse
	_delay_ms(2); // Wait for command to be processed
}

void lcd_data(char data){
	PORTC |= (1 << RS); // RS = 1 for data
	PORTD = data; // Put data on data bus
	PORTC |= (1 << E); // Enable pulse
	_delay_us(5);
	PORTC &= ~(1 << E); // Disable pulse
	_delay_ms(2); // Wait for data to be processed
}

void lcd_print(char *str) {
	while (*str) {
		lcd_data(*str++); // Send characters one by one
	}
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
	uint8_t address = (row == 0 ? 0x80 : 0xC0) + col; // Calculate address
	lcd_cmd(address); // Send command to set cursor
}

void int_to_string_fixed(char* buffer, int value, uint8_t width) {
	if (value < 0) {
		*buffer++ = '-'; // Add minus sign for negative values
		value = -value;
		width--;
	}
	
	buffer += width; // Move to the end of the buffer
	*buffer-- = '\0'; // Null-terminate the string
	
	for (int i = 0; i < width; i++) {
		if (value > 0) {
			*buffer-- = (value % 10) + '0'; // Convert digit to character
			value /= 10;
			} else {
			*buffer-- = '0'; // Fill remaining width with zeros
		}
	}
}

void display_temperatures_test(int temperature, int unused) {
	char buffer[16];
	
	lcd_cmd(0x80 | 0x00);  // Move cursor to the first line
	if (temperature >= -55 && temperature <= 125) {
		int_to_string_fixed(buffer, temperature, 4); // Convert temperature to string
		lcd_print("Temp:"); 
		lcd_print(buffer); 
		lcd_print("_C"); 
		} else {
		lcd_print("Temp:Error"); 
	}
}

uint8_t ds18b20_reset(void) {
	uint8_t i;
	DS18B20_DDR |= (1 << DS18B20_PIN); // Set as output
	_delay_us(480); // Pull low for 480 us
	DS18B20_DDR &= ~(1 << DS18B20_PIN); // Set as input
	_delay_us(60); // Wait for 60 us
	i = (DS18B20_PINR & (1 << DS18B20_PIN)); // Read the pin state
	_delay_us(420); // Wait for 420 us
	return i; // Return the pin state (0 if presence pulse detected)
}

void ds18b20_write_bit(uint8_t bit) {
	DS18B20_DDR |= (1 << DS18B20_PIN); // Set as output
	_delay_us(1); // Wait for 1 us
	if (bit) DS18B20_DDR &= ~(1 << DS18B20_PIN); // Set as input if bit is 1
	_delay_us(60); // Wait for 60 us
	DS18B20_DDR &= ~(1 << DS18B20_PIN); // Set as input
}

uint8_t ds18b20_read_bit(void) {
	uint8_t bit = 0;
	DS18B20_DDR |= (1 << DS18B20_PIN); // Set as output
	_delay_us(1); // Wait for 1 us
	DS18B20_DDR &= ~(1 << DS18B20_PIN); // Set as input
	_delay_us(14); // Wait for 14 us
	if (DS18B20_PINR & (1 << DS18B20_PIN)) bit = 1; // Read the pin state
	_delay_us(45); // Wait for 45 us
	return bit; // Return the bit value
}

void ds18b20_write_byte(uint8_t byte) {
	for (uint8_t i = 0; i < 8; i++) {
		ds18b20_write_bit(byte & 1); // Write each bit of the byte
		byte >>= 1; // Shift the byte to get the next bit
	}
}

uint8_t ds18b20_read_byte(void) {
	uint8_t byte = 0;
	for (uint8_t i = 0; i < 8; i++) {
		byte >>= 1; // Shift the byte to make space for the next bit
		if (ds18b20_read_bit()) byte |= 0x80; // Read each bit and set it in the byte
	}
	return byte; // Return the byte
}

int ds18b20_get_temp(void) {
	uint8_t temp_lsb, temp_msb;
	int16_t temp;

	if (ds18b20_reset()) { // Reset the DS18B20 sensor and check for presence
		lcd_print("DS18B20 Error"); // Print error message if no presence pulse
		return -1; // Return error code
	}
	ds18b20_write_byte(DS18B20_CMD_SKIPROM); // Skip ROM command
	ds18b20_write_byte(DS18B20_CMD_CONVERTTEMP); // Start temperature conversion
	while (!ds18b20_read_bit()); // Wait for conversion to complete

	if (ds18b20_reset()) { // Reset the DS18B20 sensor and check for presence
		lcd_print("DS18B20 Error"); // Print error message if no presence pulse
		return -1; // Return error code
	}
	ds18b20_write_byte(DS18B20_CMD_SKIPROM); // Skip ROM command
	ds18b20_write_byte(DS18B20_CMD_RSCRATCHPAD); // Read scratchpad command
	
	temp_lsb = ds18b20_read_byte(); // Read LSB of temperature
	temp_msb = ds18b20_read_byte(); // Read MSB of temperature
	
	temp = (temp_msb << 8) | temp_lsb; // Combine MSB and LSB
	return (int)(temp / 16); // Convert to Celsius and return
}

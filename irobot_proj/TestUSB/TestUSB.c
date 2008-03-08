#define USB 1
#define CR8 2
void setSerial(uint8_t com) {
	if(com == USB)
	PORTB |= 0x10;
	else if(com == CR8)
	PORTB &= ~0x10;
}
uint8_t getSerialDestination(void) {
	if (PORTB & 0x10)
	return USB;
	else
	return CR8;
}
void writeChar(char c, uint8_t com) {
	uint8_t originalDestination = getSerialDestination();
	if (com != originalDestination) {
										setSerial(com);
										delayus(200);
									}
	byteTx((uint8_t)(c));
	if (com != originalDestination) {
										setSerial(originalDestination);
										delayus(200);
									}
}
#pragma once

// polling timer
#define EVERY_MS(n) {\
	static uint32_t _next = 0; \
	uint32_t _cur = millis(); \
	if((int32_t)(_next - _cur) <= 0) { do {_next += n;} while(_next <= _cur);

#define END_EVERY_MS }}


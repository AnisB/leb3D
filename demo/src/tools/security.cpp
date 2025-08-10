// Internal includes
#include "tools/security.h"

// External includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

void __handle_fail(const char* msg, const char* file_name, int)
{
	printf("[ERROR] %s\n", msg);
	printf("Triggered at %s\n", file_name);
	#if WIN32
	__debugbreak();
	#else
	__builtin_trap();
	#endif
	exit(-1);
}

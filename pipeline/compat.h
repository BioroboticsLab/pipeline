#pragma once

#ifdef _MSC_VER
	#if _MSC_VER <= 1800
		#define THREAD_LOCAL __declspec( thread )
	#elif
		#define THREAD_LOCAL thread_local
	#endif
#else
	#define THREAD_LOCAL thread_local
#endif

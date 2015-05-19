// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * ColorDebug
 * Version: 0.10 - Add CD_FULL_FILE define. Revive CD_PERROR.
 * Version: 0.9 - Add CD_HIDE_**** defines.
 * Version: 0.8 - stderr only on warning and error.
 * Version: 0.7 - fix CD_INFO_NO_HEADER to not print header (thank you, tests).
 * Version: 0.6 - Show file name instead of full path - fix for windows.
 * Version: 0.5 - Drop CD_PERROR (recommend use of CD_ERROR_NO_HEADER instead).
 * Version: 0.4 - __func__ as __FUNCTION__ for VS2008.
 * Version: 0.3 - Added CD_*****_NO_HEADER. Added scopes for each CD_printf macro.
 * Version: 0.2 - Added CD_PERROR.
 * Version: 0.1 - Initial version.
 *
 * Copyright: UC3M 2014 (C)
 * Author:
 * <a href="http://roboticslab.uc3m.es/roboticslab/people/jg-victores">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, look for LGPL.TXT
 */

#ifndef __COLOR_DEBUG_HPP__
#define __COLOR_DEBUG_HPP__

#include <stdio.h>
#include <string>  // std::string
#include <string.h>  // strrchr

namespace ColorDebug {

//-- Fix for old Windows versions.
//-- Thanks: tomlogic @ http://stackoverflow.com/questions/2281970/cross-platform-defining-define-for-macros-function-and-func
#if defined ( WIN32 )
  #define __func__ __FUNCTION__
#endif

//-- Macros for size_t and similar across platforms.
//-- Thanks: dalle @ http://stackoverflow.com/questions/1546789/clean-code-to-printf-size-t-in-c-or-nearest-equivalent-of-c99s-z-in-c
#if defined(_MSC_VER)
  #define CD_SIZE_T    "%Iu"
  #define CD_SSIZE_T   "%Id"
  #define CD_PTRDIFF_T "%Id"
#elif defined(__GNUC__)
  #define CD_SIZE_T    "%zu"
  #define CD_SSIZE_T   "%zd"
  #define CD_PTRDIFF_T "%zd"
#else
  // TODO figure out which to use.
  #if NUMBITS == 32
    #define CD_SIZE_T_SPECIFIER    something_unsigned
    #define CD_SSIZE_T_SPECIFIER   something_signed
    #define CD_PTRDIFF_T_SPECIFIER something_signed
  #else
    #define CD_SIZE_T_SPECIFIER    something_bigger_unsigned
    #define CD_SSIZE_T_SPECIFIER   something_bigger_signed
    #define CD_PTRDIFF_T_SPECIFIER something-bigger_signed
  #endif
#endif

//-- Show file name instead of full path.
//-- Thanks: red1ynx @ http://stackoverflow.com/questions/8487986/file-macro-shows-full-path
#if defined ( WIN32 )  //-- For Windows use '\\' instead of '/'.
  #define __REL_FILE__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#else
  #define __REL_FILE__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#endif

//-- Color defines.
//-- Thanks: http://stackoverflow.com/questions/1961209/making-some-text-in-printf-appear-in-green-and-red
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */


//-- ------------------------ \begin Real macros ------------------------ --//

//-- CD_**** defines.
//-- Thanks: http://en.wikipedia.org/wiki/Variadic_macro
//-- Thanks: http://stackoverflow.com/questions/15549893/modify-printfs-via-macro-to-include-file-and-line-number-information
#if defined ( CD_HIDE_ERROR )
    #define CD_ERROR(...)
    #define CD_PERROR(...)
#else
    #if defined ( CD_FULL_FILE )
        #define CD_ERROR(...) {fprintf(stderr,RED); do{fprintf(stderr, "[error] %s:%d %s(): ", __FILE__, __LINE__, __func__); \
            fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr,RESET);}
        #define CD_PERROR(...) {fprintf(stderr,RED); do{fprintf(stderr, "[error] %s:%d %s(): ", __FILE__, __LINE__, __func__); \
            fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr, "[error] "); perror(""); fprintf(stderr,RESET);}
    #else
        #define CD_ERROR(...) {fprintf(stderr,RED); do{fprintf(stderr, "[error] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
            fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr,RESET);}
        #define CD_PERROR(...) {fprintf(stderr,RED); do{fprintf(stderr, "[error] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
            fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr, "[error] "); perror(""); fprintf(stderr,RESET);}
    #endif
#endif

#if defined ( CD_HIDE_WARNING )
    #define CD_WARNING(...)
#else 
    #if defined ( CD_FULL_FILE )
        #define CD_WARNING(...) {fprintf(stderr,YELLOW); do{fprintf(stderr, "[warning] %s:%d %s(): ", __FILE__, __LINE__, __func__); \
            fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr,RESET);}
    #else
        #define CD_WARNING(...) {fprintf(stderr,YELLOW); do{fprintf(stderr, "[warning] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
            fprintf(stderr, __VA_ARGS__);} while(0); fprintf(stderr,RESET);}
    #endif
#endif

#if defined ( CD_HIDE_SUCCESS )
    #define CD_SUCCESS(...)
#else 
    #if defined ( CD_FULL_FILE )
        #define CD_SUCCESS(...) {printf(GREEN); do{printf("[success] %s:%d %s(): ", __FILE__, __LINE__, __func__); \
            printf(__VA_ARGS__);} while(0); printf(RESET);}
    #else
        #define CD_SUCCESS(...) {printf(GREEN); do{printf("[success] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
            printf(__VA_ARGS__);} while(0); printf(RESET);}
    #endif
#endif

#if defined ( CD_HIDE_INFO )
    #define CD_INFO(...)
#else 
    #if defined ( CD_FULL_FILE )
        #define CD_INFO(...) {do{printf("[info] %s:%d %s(): ", __FILE__, __LINE__, __func__); \
            printf(__VA_ARGS__);} while(0);}
    #else
        #define CD_INFO(...) {do{printf("[info] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
            printf(__VA_ARGS__);} while(0);}
    #endif
#endif

#if defined ( CD_HIDE_DEBUG )
    #define CD_DEBUG(...)
#else 
    #if defined ( CD_FULL_FILE )
        #define CD_DEBUG(...) {printf(BLUE); do{printf("[debug] %s:%d %s(): ", __FILE__, __LINE__, __func__); \
            printf(__VA_ARGS__);} while(0); printf(RESET);}
    #else
        #define CD_DEBUG(...) {printf(BLUE); do{printf("[debug] %s:%d %s(): ", __REL_FILE__, __LINE__, __func__); \
            printf(__VA_ARGS__);} while(0); printf(RESET);}
    #endif
#endif

//-- CD_****_NO_HEADER defines.
#if defined ( CD_HIDE_ERROR )
    #define CD_ERROR_NO_HEADER(...)
#else 
    #define CD_ERROR_NO_HEADER(...) {fprintf(stderr,RED); fprintf(stderr, __VA_ARGS__); fprintf(stderr,RESET);}
#endif

#if defined ( CD_HIDE_WARNING )
    #define CD_WARNING_NO_HEADER(...)
#else 
    #define CD_WARNING_NO_HEADER(...) {fprintf(stderr,YELLOW); fprintf(stderr, __VA_ARGS__); fprintf(stderr,RESET);}
#endif

#if defined ( CD_HIDE_SUCCESS )
    #define CD_SUCCESS_NO_HEADER(...)
#else 
    #define CD_SUCCESS_NO_HEADER(...) {printf(GREEN); printf(__VA_ARGS__); printf(RESET);}
#endif

#if defined ( CD_HIDE_INFO )
    #define CD_INFO_NO_HEADER(...)
#else 
    #define CD_INFO_NO_HEADER(...) {printf(__VA_ARGS__);}
#endif

#if defined ( CD_HIDE_DEBUG )
    #define CD_DEBUG_NO_HEADER(...)
#else 
    #define CD_DEBUG_NO_HEADER(...) {printf(BLUE); printf(__VA_ARGS__); printf(RESET);}
#endif

//-- ------------------------ \end Real macros ------------------------ --//

} //ColorDebug

#endif  // __COLOR_DEBUG_HPP__


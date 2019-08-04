

#ifndef _predicatesdllexport_h_
#define _predicatesdllexport_h_


#if defined (_MSC_VER)
//#pragma warning(disable: 4267)
//#pragma warning(disable: 4800) /*  warning C4800: 'double' : forcing value to bool 'true' or 'false' */
#endif


/* Cmake will define predicates_EXPORTS on Windows when it
configures to build a shared library. If you are going to use
another build system on windows or create the visual studio
projects by hand you need to define predicates_EXPORTS when
building the predicates DLL on windows.
*/


#if defined (predicates_EXPORTS)  /* Compiling the predicates DLL/Dylib */
  #if defined (_MSC_VER)  /* MSVC Compiler Case */
    #define  PREDICATES_EXPORT __declspec(dllexport)
    #define EXPIMP_TEMPLATE
  #elif (__GNUC__ >= 4)  /* GCC 4.x has support for visibility options */
    #define PREDICATES_EXPORT __attribute__ ((visibility("default")))
  #endif
#else  /* Importing the DLL into another project */
  #if defined (_MSC_VER)  /* MSVC Compiler Case */
    #define  PREDICATES_EXPORT __declspec(dllimport)
    #define EXPIMP_TEMPLATE extern
  #elif (__GNUC__ >= 4)  /* GCC 4.x has support for visibility options */
    #define PREDICATES_EXPORT __attribute__ ((visibility("default")))
  #endif
#endif


/* If PREDICATES_EXPORT was never defined, define it here */
#ifndef PREDICATES_EXPORT
  #define PREDICATES_EXPORT
  #define EXPIMP_TEMPLATE
#endif

#endif /*  */


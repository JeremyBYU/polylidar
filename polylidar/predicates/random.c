
#include <stdlib.h>

/*****************************************************************************/
/*                                                                           */
/*  doublerand()   Generate a double with random 53-bit significand and a    */
/*                 random exponent in [0, 511].                              */
/*                                                                           */
/*****************************************************************************/

double doublerand()
{
  int a = rand();
  int b = rand();
  int c = rand();
  double expo = 2;
  double result = (double) (a - 1073741824) * 8388608.0 + (double) (b >> 8);
  for (int i = 512; i <= 131072; i *= 2, expo = expo * expo) {
    if (c & i) {
      result *= expo;
    }
  }
  return result;
}

/*****************************************************************************/
/*                                                                           */
/*  narrowdoublerand()   Generate a double with random 53-bit significand    */
/*                       and a random exponent in [0, 7].                    */
/*                                                                           */
/*****************************************************************************/

double narrowdoublerand()
{
  int a = rand();
  int b = rand();
  int c = rand();
  double expo = 2;
  double result = (double) (a - 1073741824) * 8388608.0 + (double) (b >> 8);
  for (int i = 512; i <= 2048; i *= 2, expo = expo * expo) {
    if (c & i) {
      result *= expo;
    }
  }
  return result;
}

/*****************************************************************************/
/*                                                                           */
/*  uniformdoublerand()   Generate a double with random 53-bit significand.  */
/*                                                                           */
/*****************************************************************************/

double uniformdoublerand()
{
  int a = rand();
  int b = rand();
  double result = (double) (a - 1073741824) * 8388608.0 + (double) (b >> 8);
  return result;
}

/*****************************************************************************/
/*                                                                           */
/*  floatrand()   Generate a float with random 24-bit significand and a      */
/*                random exponent in [0, 63].                                */
/*                                                                           */
/*****************************************************************************/

float floatrand()
{
  int a = rand();
  int c = rand();
  float expo = 2;
  float result = (float) ((a - 1073741824) >> 6);
  for (int i = 512; i <= 16384; i *= 2, expo = expo * expo) {
    if (c & i) {
      result *= expo;
    }
  }
  return result;
}

/*****************************************************************************/
/*                                                                           */
/*  narrowfloatrand()   Generate a float with random 24-bit significand and  */
/*                      a random exponent in [0, 7].                         */
/*                                                                           */
/*****************************************************************************/

float narrowfloatrand()
{
  int a = rand();
  int c = rand();
  float expo = 2;
  float result = (float) ((a - 1073741824) >> 6);
  for (int i = 512; i <= 2048; i *= 2, expo = expo * expo) {
    if (c & i) {
      result *= expo;
    }
  }
  return result;
}

/*****************************************************************************/
/*                                                                           */
/*  uniformfloatrand()   Generate a float with random 24-bit significand.    */
/*                                                                           */
/*****************************************************************************/

float uniformfloatrand()
{
  int a = rand();
  float result = (float) ((a - 1073741824) >> 6);
  return result;
}

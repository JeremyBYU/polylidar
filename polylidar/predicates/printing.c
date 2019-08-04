
#include <stddef.h>
#include <stdio.h>

/**
 * Print the bit representation of a double.
 */
void print_double(double number)
{
  unsigned long long no;
  unsigned long long sign, expo;
  int exponent;
  int bottomi;

  no = *(unsigned long long *) &number;
  sign = no & 0x8000000000000000ll;
  expo = (no >> 52) & 0x7ffll;
  exponent = (int) expo;
  exponent = exponent - 1023;

  if (sign)
    printf("-");
  else
    printf(" ");

  if (exponent == -1023) {
    printf("0.0000000000000000000000000000000000000000000000000000_     (   )");
  } else {
    printf("1.");
    bottomi = -1;
    for (int i = 0; i < 52; i++) {
      if (no & 0x0008000000000000ll) {
        printf("1");
        bottomi = i;
      } else {
        printf("0");
      }
      no <<= 1;
    }
    printf("_%d  (%d)", exponent, exponent - 1 - bottomi);
  }
}


/**
 * Print the bit representation of a float.
 */
void print_float(float number)
{
  unsigned no;
  unsigned sign, expo;
  int exponent;
  int bottomi;

  no = *(unsigned *) &number;
  sign = no & 0x80000000;
  expo = (no >> 23) & 0xff;
  exponent = (int) expo;
  exponent = exponent - 127;
  if (sign) {
    printf("-");
  } else {
    printf(" ");
  }
  if (exponent == -127) {
    printf("0.00000000000000000000000_     (   )");
  } else {
    printf("1.");
    bottomi = -1;
    for (int i = 0; i < 23; i++) {
      if (no & 0x00400000) {
        printf("1");
        bottomi = i;
      } else {
        printf("0");
      }
      no <<= 1;
    }
    printf("_%3d  (%3d)", exponent, exponent - 1 - bottomi);
  }
}


/**
 * Print the bit representation of an expansion.
 */
void print_expansion(size_t elen, double * e)
{
  for (int i = elen - 1; i >= 0; i--) {
    print_double(e[i]);
    if (i > 0)
      printf(" +\n");
    else
      printf("\n");
  }
}

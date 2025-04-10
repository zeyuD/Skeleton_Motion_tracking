/* Copyright (C) 2012-2017 Ultraleap Limited. All rights reserved.
 *
 * Use of this code is subject to the terms of the Ultraleap SDK agreement
 * available at https://central.leapmotion.com/agreements/SdkAgreement unless
 * Ultraleap has signed a separate license agreement with you or your
 * organisation.
 *
 */

#include "LeapC.h"

#include <stdio.h>
#include <stdlib.h>

#define LEAPC_CHECK(func)                                                 \
  do                                                                      \
  {                                                                       \
    eLeapRS result = func;                                                \
    if (result != eLeapRS_Success)                                        \
    {                                                                     \
      printf("Fatal error in calling function: %s: %X\n", #func, result); \
      abort();                                                            \
    }                                                                     \
  } while (0)

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "Usage: %s <flag>\n", argv[0]);
    return -1;
  }

  LEAP_CONNECTION connection;

  LEAPC_CHECK(LeapCreateConnection(NULL, &connection));
  LEAPC_CHECK(LeapOpenConnection(connection));

  LEAP_CONNECTION_MESSAGE msg;
  unsigned int timeout_ms = 1000;
  bool is_enabled = false;
  eLeapRS res = eLeapRS_NotAvailable;

  do
  {
    LeapPollConnection(connection, timeout_ms, &msg);
    res = LeapCheckLicenseFlag(connection, argv[1], &is_enabled);
  } while (res != eLeapRS_Success);

  printf("Flag \"%s\" %s enabled\n", argv[1], (is_enabled ? "is" : "is not"));

  LeapCloseConnection(connection);
  LeapDestroyConnection(connection);

  return 0;
}

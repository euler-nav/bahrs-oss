/**
 * @file {CLASS_NAME}.cpp
 * @brief Implementation of the runtime environment class.
 *
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date {DATE_GENERATED}
 */

#include "{CLASS_NAME}.h"

{CLASS_NAME}& {CLASS_NAME}::GetInstance()
{
  static CRte soRteInstance;
  return soRteInstance;
}

{CLASS_NAME}::{CLASS_NAME}()
{
}

void {CLASS_NAME}::Init()
{
{INITIALIZATION_OF_PORTS}
{DEFAULT_DEBUG_OUTPUT}
}


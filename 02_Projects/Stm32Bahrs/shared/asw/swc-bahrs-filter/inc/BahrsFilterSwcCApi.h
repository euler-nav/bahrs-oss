/**
 * @file BahrsFilterSwcCApi.h
 * @brief Declaration C APIs of the BAHRS filter SWC.
 * @author Fedor Baklanov
 * @date 4 November 2024
 */

#ifndef BAHRS_FILTER_SWC_C_API_H
#define BAHRS_FILTER_SWC_C_API_H

#ifdef __cplusplus
  #error This header must not be included in .cpp files.
#endif

#include <stdint.h>

/**
 * @brief A C-wrapper of CBahrsFilterSwc::SetImuInput().
 * @param uFilterIndex Index of the filter SWC instance.
 */
void BahrsFilterSwcSetImuInput(uint32_t uFilterIndex);

#endif /* BAHRS_FILTER_SWC_C_API_H */


/**
 * @file RteWrapper.h
 * @brief Declaration of C-wrappers of the CRte class.
 * @author Fedor Baklanov
 * @date 04 February 2022
 */

#ifndef RTE_WRAPPER_H
#define RTE_WRAPPER_H

/**
 * A wrapper function declared as extern "C". Calls CRte::Init().
 */
void RteInit();

#endif /* RTE_WRAPPER_H */

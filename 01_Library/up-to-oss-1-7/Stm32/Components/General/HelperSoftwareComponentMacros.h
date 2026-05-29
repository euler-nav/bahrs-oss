/**
 * @file HelperSoftwareComponentMacros.h
 * @brief Implementation of auxiliary convenience macros.
 * @author Fedor Baklanov
 * @date 26 May 2023
 */

#ifndef HELPER_SOFTWARE_COMPONENT_MACROS_H
#define HELPER_SOFTWARE_COMPONENT_MACROS_H

#define FORBID_CLASS_COPY_AND_MOVE(CClassName) \
CClassName(CClassName&) = delete; \
CClassName(CClassName&&) = delete; \
CClassName& operator=(const CClassName& orOther) = delete; \
CClassName& operator=(CClassName&& orOther) = delete;

#define DECLARE_MANDATORY_APIS(CSwcClass) \
public: \
  void Init() override; \
  bool IsInitialized() override; \
  \
private: \
  static CSwcClass& getInstanceImpl(unsigned uInstanceIndex);


#endif /* HELPER_SOFTWARE_COMPONENT_MACROS_H */

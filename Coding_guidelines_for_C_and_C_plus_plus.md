C/C++ Coding Guidelines

BAHRS project


Version 0.1

*© 2022 Fedor Baklanov. All rights reserved.*


**Table of contents**

[1**	**Introduction	3](#_Toc95659842)

[2**	**Glossary	4](#_Toc95659843)

[2.1**	**Terms and definitions	4](#_Toc95659844)

[2.2**	**Abbreviations	4](#_Toc95659845)



# Introduction

# General

## Source code formatting

### Tabs SHALL NOT be used.

Note: whitespaces shall be used instead of tabs.

### Indentation SHALL be of width 2.

### Braces SHALL be placed to a separate line.

Exception: initialization lists.

### In variable declarations the symbols “\*” and “&” SHALL be placed after the type specifier without a space.

### C-style type casts SHALL NOT be used in C++ source and header files.

### Implicit casts from integer to bool type SHALL NOT be used.

### Implicit casts from bool to integer type SHALL NOT be used.

## Naming convention

### Variable, function, class, and template names SHALL follow the camel-case style.

### Names of public functions and public class methods SHALL start with a capital letter.

### Names of private functions and private and protected class methods SHALL start with a lowercase letter.

### Class names SHALL be prefixed with “C”.

Example: *CMyClass*.

### Structure names SHALL be prefixed with “S”.

Example: *SMyStruct*.

### All the members of a structure SHALL be public.

### Enumerator names SHALL be prefixed with “E”.

Example: *EMyEnum*.

### Class member variable names SHALL end with the underscore symbol “\_”.

### Enum class member names SHALL NOT have a trailing underscore.

### Variable names SHALL be prefixed by a code describing variable type.

### A prefix of a variable name SHALL be in lowercase.

### The variable prefix code SHALL consist of up to five letters placed in the sequence explained below.

1. Variable specifier: “s” for static, “v” for volatile

2. Constness: “k” for const variables, nothing otherwise

3. Array: „a“ for naked arrays, nothing otherwise.

4. Variable type:

   a. “i” for signed integers

   b. “u” for unsigned integers

   c. “b” for Boolean

   d. “f” for 32 bit floating point numbers

   e. “d” for 64 bit floating point numbers

   f. “c” for characters

   g. “o” for classes and structures

   h. “e” for enums

   i. Nothing for void

5. Pointer or reference: “p” for a pointer, “r” for a reference

Example:

static const int skiMyInteger;

unsigned int\* upMyPointerToInteger;

CMyClass oMyClassInstance;

void\* pMyVoidPointer;


### File names SHALL be in camel-case.

### Multiple inclusion guard SHALL be in capital letters. Single words SHALL be separated by the underscore “\_”.

Example (file name CMyClass.h):

	\#ifndef C\_MY\_CLASS\_H

	\#define C\_MY\_CLASS\_H

	…

# Glossary

## Terms and definitions

## Abbreviations

﻿#ifndef MOTORSCONTROLL_GLOBAL_H
#define MOTORSCONTROLL_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(ACTUATORCONTROLLER_LIBRARY)
#  define ACTUATORCONTROLLERSHARED_EXPORT Q_DECL_EXPORT
#else
#  define ACTUATORCONTROLLERSHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // MOTORSCONTROLL_GLOBAL_H

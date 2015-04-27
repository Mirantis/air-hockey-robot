//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This file is part of CL-EyeMulticam SDK
//
// C++ Sample Application
//
// It allows the use of multiple CL-Eye cameras in your own applications
//
// For updates and file downloads go to: http://codelaboratories.com/research/view/cl-eye-muticamera-sdk
//
// Copyright 2008-2012 (c) Code Laboratories, Inc. All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once

#include "targetver.h"

#include <conio.h>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>

#include "CLEyeMulticam.h"
#include "opencv.hpp"

static LARGE_INTEGER _frequencyPT;

static inline PVOID ProfileMSStart()
{
	PLARGE_INTEGER start = new LARGE_INTEGER;
	QueryPerformanceFrequency(&_frequencyPT);
	QueryPerformanceCounter(start);
	return (PVOID)start;
}

static inline double ProfileMSEnd(PVOID p)
{
	LARGE_INTEGER stop, diff;
	PLARGE_INTEGER start = (PLARGE_INTEGER)p;
	QueryPerformanceCounter(&stop);
	diff.QuadPart = stop.QuadPart - start->QuadPart;
	double timeMs = 1000.0 * ((double)diff.QuadPart / (double)_frequencyPT.QuadPart);
	delete start;
	return timeMs;
}

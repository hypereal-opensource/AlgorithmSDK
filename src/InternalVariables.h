/*
 * The MIT License (MIT)
 * Copyright (c) 2017 Shanghai Chai Ming Huang Info&Tech Co£¬Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#pragma once
#ifdef ALGORITHMDLL_EXPORTS
#define ALGORITHMDLL_API __declspec(dllexport) 
#else
#define ALGORITHMDLL_API __declspec(dllimport) 
#endif

#include <vector>
#include <string>
#include "mutex"
#include <map>


namespace HYPEREAL
{
	/************************************************************************
	* Read internal.txt file, as internal variables.
	* Now support string key, ands string/int value.
	* If a key starts with '#', the KV will be ignored, it is NOT a comment feature!
	************************************************************************/
	class InternalVariables
	{
		static std::recursive_mutex mtx;
		static InternalVariables * singletonInstance;
		InternalVariables(std::string workspacePath, bool enableLogging);
		bool enableLogging;
	public:
		static void Initialize(std::string workspacePath, bool enableLogging);
		static InternalVariables * GetInstance();
		std::string Get(std::string key, std::string defaultValue);
		void Set(std::string key, std::string value);
		int GetInt(std::string key, int defaultValue);
		double GetDouble(std::string key, double defaultValue);
		bool GetBool(std::string key, bool defaultValue);
		bool containsKey(std::string key);
		std::string GetWorkspace();
	private:
		InternalVariables();
		void LoadVariables();
		std::map<std::string, std::string> attrMap;
	};

	// For exporting to DV
	class ALGORITHMDLL_API InternalVarExport
	{
	public:
		static void Initialize(const char* workspacePath, bool enableLogging);
		static void Set(const char* key, const char* value);
		static void Get(const char* key, const char* defaultValue, char* outValue);
		static std::string Get(const char* key, const char* defaultValue);
		static int GetInt(const char* key, int defaultValue);
		static bool GetBool(const char* key, bool defaultValue);
		static double GetDouble(const char* key, double defaultValue);
		static bool containsKey(const char* key);
	};

}

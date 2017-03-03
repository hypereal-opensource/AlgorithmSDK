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

#include <mutex>
#include <fstream>
#include <sstream>
#include "InternalVariables.h"
#include "LogUtil.h"


namespace HYPEREAL
{
	using std::string;
	std::recursive_mutex InternalVariables::mtx;
	InternalVariables * InternalVariables::singletonInstance = NULL;

	void InternalVariables::Initialize(std::string workspacePath, bool enableLogging)
	{
		std::lock_guard<std::recursive_mutex> lock(mtx);
		singletonInstance = new InternalVariables(workspacePath, enableLogging);
		singletonInstance->LoadVariables();
	}

	InternalVariables* InternalVariables::GetInstance()
	{
		std::lock_guard<std::recursive_mutex> lock(mtx);
		if (!singletonInstance)
		{
			// If GetInstance before initializing, set it as relative path.
			singletonInstance = new InternalVariables(GetUserWorkspace(), false);
			singletonInstance->LoadVariables();
		}
		return singletonInstance;
	}

	InternalVariables::InternalVariables(std::string workspacePath, bool enableLogging)
		: enableLogging(enableLogging)
	{
		attrMap["PATH_WORKSPACE"] = workspacePath;
	}

	void InternalVariables::LoadVariables()
	{
		// Because of recursive dependency, this function won't use LogUtil to do logging.
		std::stringstream ss;

		const int LINE_LENGTH = 1024;
		char str[LINE_LENGTH];
		std::string key = "";
		std::string val = "";
		std::vector<std::string> fileNames({ "internal.txt", "define.txt" });

		for each (std::string filename in fileNames)
		{
			std::string path = attrMap["PATH_WORKSPACE"] + filename;
			std::ifstream fin(path);
			ss << ("Start load internal variables at " + path + "\n");
			while (fin.getline(str, LINE_LENGTH))
			{
				std::string line(str);
				if (line.length() == 0)
				{
					continue;
				}
				if (line[0] == '#')
				{
					ss << "Ignore line: [" + line + "]\n";
					continue;
				}
				size_t spacePos = line.find(" ");
				if (spacePos == std::string::npos)
				{
					ss << "Error line, no space: [" + line + "]\n";
				}
				key = line.substr(0, spacePos);
				val = line.substr(spacePos + 1);

				if (filename == "define.txt")
				{
					// For define.txt to override internal. Useful for old records.
					if (key.find("internal.") == 0)
					{
						key = key.substr(strlen("internal."));
					}
					else
					{
						key = "define." + key; // use define.XXXXX
					}
				}

				ss << "Load attr: [" + key + "] => [" + val + "]\n";
				attrMap[key] = val;
			}

			fin.close();
			ss << ("Done.\n\n");
		}

		if (enableLogging)
		{
			std::ofstream ofs(attrMap["PATH_WORKSPACE"] + "log-InternalVariables.txt", std::ios_base::trunc);
			ofs << ss.str();
			ofs.close();
		}
	}

	std::string InternalVariables::Get(std::string key, std::string defaultValue)
	{
		auto it = attrMap.find(key);
		if (it == attrMap.cend())
		{
			// Not found.
			return defaultValue;
		}
		return it->second;
	}

	std::string InternalVariables::GetWorkspace()
	{
		return Get("PATH_WORKSPACE", GetUserWorkspace());
	}

	void InternalVariables::Set(std::string key, std::string value)
	{
		attrMap[key] = value;
		LogUtil::getLogger("InternalVariables")
			->log("Override attr: [" + key + "] => [" + value + "]\n");
	}

	int InternalVariables::GetInt(std::string key, int defaultValue)
	{
		auto it = attrMap.find(key);
		if (it == attrMap.cend())
		{
			// Not found.
			return defaultValue;
		}
		return std::stoi(it->second);
	}

	double InternalVariables::GetDouble(std::string key, double defaultValue)
	{
		auto it = attrMap.find(key);
		if (it == attrMap.cend())
		{
			// Not found.
			return defaultValue;
		}
		return std::stod(it->second);
	}

	bool InternalVariables::GetBool(std::string key, bool defaultValue)
	{
		auto it = attrMap.find(key);
		if (it == attrMap.cend())
		{
			// Not found.
			return defaultValue;
		}
		return it->second == "true" || it->second == "1";
	}


	bool InternalVariables::containsKey(std::string key)
	{
		return attrMap.find(key) == attrMap.cend();
	}

	//////////////////////////////////////////////////////////////////////////
	// InternalVarExport
	//////////////////////////////////////////////////////////////////////////
	void InternalVarExport::Initialize(const char* workspacePath, bool enableLogging)
	{
		InternalVariables::Initialize(workspacePath, enableLogging);
	}
	void InternalVarExport::Set(const char* key, const char* value)
	{
		InternalVariables::GetInstance()->Set(key, value);
	}
	void InternalVarExport::Get(const char* key, const char* defaultValue, char outValue[128])
	{
		std::string v = InternalVariables::GetInstance()->Get(key, defaultValue);
		strncpy_s(outValue, 128,v.substr(0, 127).c_str(), 128);
	}
	std::string InternalVarExport::Get(const char* key, const char* defaultValue)
	{
		return InternalVariables::GetInstance()->Get(key, defaultValue);
	}
	int InternalVarExport::GetInt(const char* key, int defaultValue)
	{
		return InternalVariables::GetInstance()->GetInt(key, defaultValue);
	}
	double InternalVarExport::GetDouble(const char* key, double defaultValue)
	{
		return InternalVariables::GetInstance()->GetDouble(key, defaultValue);
	}
	bool InternalVarExport::GetBool(const char* key, bool defaultValue)
	{
		return InternalVariables::GetInstance()->GetBool(key, defaultValue);
	}
	bool InternalVarExport::containsKey(const char* key)
	{
		return InternalVariables::GetInstance()->containsKey(key);
	}
}

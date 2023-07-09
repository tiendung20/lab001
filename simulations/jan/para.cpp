#include <omp.h>
#include <unistd.h>
#include <dirent.h>
#include <cstring>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>

#include "boost/lexical_cast.hpp"
using boost::lexical_cast;

using namespace std;

enum STATE_OF_REQUEST
{
	WAITING_FOR_PROCESSING,
	BEING_PROCESSED,
	FINISHED,
	NONE
};

template <typename Numeric, typename Generator = std::mt19937>
Numeric random(Numeric from, Numeric to)
{
	thread_local static Generator gen(std::random_device{}());

	using dist_type = typename std::conditional<
		std::is_integral<Numeric>::value, std::uniform_int_distribution<Numeric>, std::uniform_real_distribution<Numeric>>::type;

	thread_local static dist_type dist;

	return dist(gen, typename dist_type::param_type{from, to});
}

std::string exec(const char *cmd)
{
	char buffer[128];
	std::string result = "";
	FILE *pipe = popen(cmd, "r");
	if (!pipe)
		throw std::runtime_error("popen() failed!");
	try
	{
		while (fgets(buffer, sizeof buffer, pipe) != NULL)
		{
			result += buffer;
		}
	}
	catch (...)
	{
		pclose(pipe);
		throw;
	}
	pclose(pipe);
	return result;
}

std::vector<std::string> getInput()
{
	std::string inputPath = "find input/ -type f -exec basename {} ';'";
	std::string fileName = exec(inputPath.c_str());
	fileName = fileName.substr(0, fileName.size() - 1); // remove newline character at the end of line

	std::vector<std::string> s;
	if (fileName.length() > 0)
	{
		size_t pos = 0;
		std::string token;
		std::string delimiter = std::string("\n");
		while ((pos = fileName.find(delimiter)) != std::string::npos)
		{
			token = fileName.substr(0, pos);
			s.push_back(token);
			fileName.erase(0, pos + delimiter.length());
		}
		s.push_back(fileName);

		std::system("rm input/*");
	}

	return s;
}

std::string getVehicleId(std::string data)
{
	std::string delimiter = "_";
	return data.substr(0, data.find(delimiter));
}

bool fileExists(std::string vehicleId)
{
	std::string inputPath = "find output/ -name " + vehicleId + "_* -exec basename {} ';' -quit";
	std::string fileName = exec(inputPath.c_str());
	if (fileName.length() == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

int main()
{
	cout << "---[Program started]---" << endl;
	STATE_OF_REQUEST state = WAITING_FOR_PROCESSING;

	// int action = 0;

	while (1)
	{
		usleep(100000);
		std::vector<std::string> dataInput;
		switch (state)
		{
		case BEING_PROCESSED:
			break;
		case WAITING_FOR_PROCESSING:
			state = BEING_PROCESSED;
			dataInput = getInput();
			if (dataInput.size() > 0)
			{
				for (std::string temp : dataInput)
				{
					std::string vehicleId = getVehicleId(temp);
					if (!fileExists(vehicleId))
					{
						double waitingTime = random<double>(20.0, 30.0);
						std::string waitingTimeStr = lexical_cast<std::string>(waitingTime);
						std::ofstream(std::string("output/") + vehicleId + "_" + waitingTimeStr);
						cout << "1 request has been processed: ";
						cout << vehicleId + "_" + waitingTimeStr << endl;
						cout << "Continue run at: " + lexical_cast<std::string>(waitingTime + 57.390) << endl;
						cout << "Waiting for new request..." << endl;
					}
				}
			}

			state = FINISHED;
			break;
		case FINISHED:
			state = WAITING_FOR_PROCESSING;
			// cout << "Waiting for new order" << endl;
			break;
		}
	}
	cout << "End" << endl;
	// closedir(folder);
	return 0;
}
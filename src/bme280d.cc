#include <fcntl.h>
#include <sys/stat.h>
#include <semaphore.h>

#include <syslog.h>
#include <unistd.h>
#include <getopt.h>

#include <ctime>
#include <string>
#include <vector>
#include <stdexcept>

#include <wiringPi.h>

using namespace std;

#include "config.h"
#include "BME280.h"
#include "SysSem.h"

struct Receiver {
	Receiver(BME280 *receiver, SysSem *semaphore, string *jsonFileName)
		: receiver(receiver), semaphore(semaphore), jsonFileName(jsonFileName) {}
	BME280 *receiver;
	SysSem *semaphore;
	string *jsonFileName;
};
static vector<Receiver *> receivers;
static void jsonUpdateLoop() {
	for (Receiver * r : receivers) {
		char newFile[PATH_MAX];
		snprintf(newFile, sizeof newFile, "%s.new", r->jsonFileName->c_str());

		FILE *json = fopen(newFile, "w");
		if (json == NULL) {
			syslog(LOG_ERR, "Can't open JSON file: %s", newFile);
		} else {
			fprintf(json, "{%s}\n", r->receiver->toJSON());
			fsync(fileno(json));
			fclose(json);
			rename(newFile, r->jsonFileName->c_str());
			syslog(LOG_MAKEPRI(LOG_USER, LOG_INFO), "%s", r->receiver->toString());
		}
	}
}
int main(int argc, char *argv[]) {
	SysSem	semaphore("/i2c");

	openlog (semaphore.imgName(),
			LOG_PERROR|LOG_CONS|LOG_PID|LOG_NDELAY, LOG_USER);
	syslog(LOG_MAKEPRI(LOG_USER, LOG_INFO), "%s", PACKAGE_STRING);

	I2CSETUP	i2c;

	i2c.channel = 0x01;
	i2c.address = 0x76;
	int polltime = 30;

	if (wiringPiSetupSys() == -1) {
		const char * error = "Can't setup wiringPi in Sys mode";
		syslog(LOG_MAKEPRI(LOG_USER, LOG_ERR), "%s", error);
		throw invalid_argument(error);
	}

	while (true) {
		static const struct option long_options[] = {
			{"channel",	required_argument, 0, 'c' },
			{"address",	required_argument, 0, 'a' },
			{"polltime",	required_argument, 0, 'p' },
			{"jsonfile",	required_argument, 0, 'j' },
			{"help",		no_argument,       0, 'h' },
			{0,         	0,                 0,   0 }
		};
		int c = getopt_long(argc, argv, "c:a:p:j:h", long_options, NULL);
		if (c == -1) {
			break;
		}
		switch (c) {
			case 'c':
				i2c.channel = strtol(optarg, NULL, 0);
				break;
			case 'a':
				i2c.address = strtol(optarg, NULL, 0);
				break;
			case 'p':
				polltime = strtol(optarg, NULL, 0);
				break;
			case 'j':
				receivers.push_back(new Receiver(
					new BME280(i2c), &semaphore, new string(optarg)));
				break;
			case 'h':
			default:
				printf("Usage: %s --polltime arg", argv[0]);
				printf(" --channel arg --address arg --jsonfile arg\n");
				printf("\t[--channel arg --address arg --jsonfile arg]\n");
				exit(0);
		}
	}
	try {
		while (receivers.size() != 0) {
			for (Receiver * r : receivers) {
				semaphore.lock();
				r->receiver->acquireData();
				semaphore.unlock();
			}
			jsonUpdateLoop();
			sleep(polltime);
		}
	} catch (exception& e) {
		semaphore.unlock();
		syslog(LOG_MAKEPRI(LOG_USER, LOG_ERR), "%s", e.what());
		throw;
	}
	return 0;
}

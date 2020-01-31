CC=g++ -fopenmp
FLAGS=-std=c++17 -pipe
OPTIMIZATION=-march=native -O3
DEBUG=-rdynamic -Werror -Wpedantic -pedantic-errors -Wall -Wextra -Wparentheses -Wfatal-errors -D_GLIBCXX_DEBUG -g -fno-omit-frame-pointer
RELEASE=-ffast-math -ftree-vectorize -Wfatal-errors -DNDEBUG


all: BikeSharing BikeSharingRelease

BikeSharing:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(DEBUG) -o BikeSharing BikeSharing.cpp

BikeSharingRelease:
	$(CC) $(FLAGS) $(OPTIMIZATION) $(RELEASE) -o BikeSharing BikeSharing.cpp


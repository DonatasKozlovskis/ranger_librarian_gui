#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <map>

using std::string;

// Struct for books
struct Book {
    string author;
    string callNumber;
    double weight;
};

// navigator ACTIONS
 enum NavigatorAction
{
    NAVIGATOR_STOP,
    NAVIGATOR_MOVE,
    NAVIGATOR_FINISH
};
static const string NavigatorActionStrings[] =
{
    "STOP",
    "MOVE",
    "FINISH"
};

// navigator ACTIONS
enum DepthLowCommand
{
    DEPTH_LOW_SLOW,
    DEPTH_LOW_STOP,
    DEPTH_LOW_STUCK
};


#endif // UTILS_H

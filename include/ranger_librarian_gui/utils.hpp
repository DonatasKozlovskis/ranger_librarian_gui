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

static const string NavigatorActionStrings[] =
{
    "I am stopped",
    "I am just moving around..",
    "Sorry, I am goint to service place"
};

#endif // UTILS_H

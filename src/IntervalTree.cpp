#include <IntervalTree.h>

const IntervalType IntervalType::CLOSED = IntervalType(false, false);
const IntervalType IntervalType::OPEN = IntervalType(true, true);
const IntervalType IntervalType::OPEN_LEFT = IntervalType(true, false);
const IntervalType IntervalType::OPEN_RIGHT = IntervalType(false, true);
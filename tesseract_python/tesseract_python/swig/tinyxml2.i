%{
#include "tinyxml2.h"
%}
%ignore tinyxml2::StrPair;
%ignore tinyxml2::DynArray;
%ignore tinyxml2::MemPool;
%include "tinyxml2.h"
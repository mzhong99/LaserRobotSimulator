#include "Utility.hpp"
#include <vector>

std::string Utility::StringFormat(std::string fmt, va_list args)
{
    int len;
    const char *cfmt;
    va_list args_tmp;
    std::vector<char> buf;
    
    cfmt = fmt.c_str();

    va_copy(args_tmp, args);

    len = vsnprintf(NULL, 0, cfmt, args_tmp);
    buf.resize(len + 1LL);

    vsnprintf(buf.data(), buf.size(), cfmt, args);

    va_end(args_tmp);

    return std::string(buf.data(), buf.size());
}

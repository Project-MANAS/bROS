# **parameter_server**
C++ wrapper around Hiredis, the official C client of the Redis database.

## Prerequisites
* [Redis >= 1.2](https://redis.io/download)
* [Hiredis](https://github.com/redis/hiredis)
## Usage
* Launch Redis server on ```127.0.0.1:6379``` 
* Import the header file into the C++ source file i.e.
``` #include "parameter_server/brosdb.h" ```
* The header file contains ```set()``` and ```get()``` functions to facilitate key-value store in the Redis database.
* The function ```set()``` is declared as follows,
```c++
template<typename T> 
void set(std::string key, T value);
```
and ```get()``` as
```c++
template<typename T> 
void get(std::string key, T *value);
```
* The key is expected to of type ```std::string```. Value can be either primitive or user defined.
## Example
```c++
#include <bits/stdc++.h>
#include "parameter_server/brosdb.h"

int main() {
  int a=5;
  std::string str("foo");
  brosdb::set(str, a);
  int b;
  brosdb::get(str, &b);
  std::cout<<b;
}
```
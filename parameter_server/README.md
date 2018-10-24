# **parameter_server**
C++ and Python wrapper to facilitate key-value store with the Redis database.
## Prerequisites
* [Redis >= 1.2](https://redis.io/download)
* [Hiredis](https://github.com/redis/hiredis)
*  [redis-py](https://github.com/andymccurdy/redis-py)
## Usage
* Launch Redis server on ```127.0.0.1:6379``` 
* C++
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
    * Source file needs to be linked against ```lhiredis```.
* Python
    * Import ```setData``` and ```getData``` from ```brosdb```
        ```python
        from brosdb import setData, getData
        ```
    * The module contains ```setData()``` and ```getData()``` functions to facilitate key-value store in the Redis database.
    * The function ```setData()``` is declared as follows, 
        ```python
        def setData(key, value)
        ```
        and ```getData()``` as
        ```python
        def getData(key, defaultValue = None)
        ```
    * The key is expected to be of type ```str```.
## Example
* C++
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
* Python
    ```python
    from brosdb import setData, getData
    
    setData("foo", "bar")
    value = getData("foo")
    print(value)
    ```

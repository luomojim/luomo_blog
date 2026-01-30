---
title: c++stl容器库学习笔记
published: 2025-11-24
pinned: false
description: 个人总结的一些c++容器库学习笔记
tags: [coding, bloging,cpp]
category: 代码
draft: false
date: 2025-11-24
pubDate: 2025-11-24
---

# c++stl的主要八种容器总结

## 1.vector容器

#### 特点：
- 实现了类似动态数组的功能，保留了数组原本的功能，同时比较方便地动态进行空间分配，内部采用迭代器进行数据的访问，同时空间在内存中也是连续的。
- 当使用push_back的时候，数据满了之后会拷贝原来的数据，同时开辟新的内存内存空间(通常是原来的两倍)来存放数据。

#### 缺陷：
- 难以在中间插入新的元素，需要把后面元素都挪完才行，容器扩容的时候需要重新分配内存。

### 代码演示

``` c++

#include <iostream>
#include <vector>

int main()
{
    int n = 10;

    // 方法1:直接初始化大小为n，值为0的容器v1。
    std::vector<int> v1(n, 0);
    // 方法2:不初始空间大小，后续再进行扩容,这种情况一定要小心越界。
    std::vector<int> v2;

    for (int i = 0; i < n; i++)
    {
        v1[i] = i;       // 像数组一样操作
        v2.push_back(i); // 在尾部添加新的元素
    }

    // 删除元素，例如下面就是删除第三个位置的内容(下标2)
    v1.erase(v1.begin() + 2);

    // 插入元素,下面的代码就是在第五个位置(下标4)插入666
    v1.insert(v1.begin() + 4, 666);

    // 修改元素，就像数组那样
    v1[5] = 777;

    // c++11(也许)新的for迭代器
    for (auto i : v1)
    {
        std::cout << i << std::endl;
    }

    // 使用迭代器进行访问，用auto类型的指针。
    // v.begin表示容器的首个位置v[0]。
    // v.end()表示容器最后一个元素的结尾！！！它没有任何的值，它用来表示容器的结尾。
    for (auto it = v1.begin(); it != v1.end(); it++)
    {
        std::cout << *it << std::endl;
    }

    // 也可以像数组一样使用下标遍历
    for (int i = 0; i < v1.size(); i++)
    {
        std::cout << v1[i] << std::endl;
    }

    return 0;
}

```

## 2.queue队列

#### 特点
- 相当于你去一饭打饭，遵循 **“先进先出”** 的道理，譬如你先排队，你就能先打饭，排队的过程就是入队，打完饭就是出队，只能操作头部数据。
- 只能访问对头和队尾数据，插入删除都在头部进行

#### 缺点
- 优点也是缺点吧，只能访问两边，中间的就不要想了，只有挨个出队出到中间才能访问

### 代码演示

``` c++

#include <iostream>
#include <queue>

int main()
{
    std::queue<int> q;

    //使用push(value)来入队
    for (int i = 0; i < 10; i++)
    {
        q.push(i);
    }

    //!q.empty()表示当队列存在的时候，同样这个函数适用于其他的stl容器
    while(!q.empty())
    {
        std::cout << q.front() << std::endl;
        q.pop();       //出队   
    }

    return 0;
}

```

## 3.deque双端队列

#### 特点
- 解决了上面单端队列的痛点，可以同时实现在**头部**或者在**尾部**插入数据。
- 可以实现类似容器的随机访问操作，也可以进行迭代器操作。

#### 缺陷
- 中间位置插入效率依旧不够快

### 代码演示

``` c++

#include <iostream>
#include <deque>

int main()
{
    std::deque<int> d1;

    // 前面的代码功能大部分与vector容器一致，这里关注deque的头尾插入
    // 从前面插入元素
    for (int i = 0; i < 3; i++)
    {
        d1.push_front(i);
    }

    for (int i = 0; i < 3; i++)
    {
        d1.push_back(i);
    }

    //与queue不同的是它可以使用迭代器进行迭代
    for (auto it = d1.begin(); it != d1.end(); it++)
    {
        std::cout << *it << std::endl;
    }

    // 也可以像数组一样使用下标遍历
    for (int i = 0; i < d1.size(); i++)
    {
        std::cout << d1[i] << std::endl;
    }

    return 0;
}

```

## 4.list双向链表

#### 特点
- 可以很快地进行数据的插入，集成了链表的功能

#### 缺点
- 和链表一样，要遍历整个链表才能获取数据的位置

### 代码演示

``` c++

#include <iostream>
#include <list>

int main()
{
    std::list<int> ls1;
    for (int i = 0; i < 3; i++)
    {
        ls1.push_front(i);
    }
    // 从尾部插入元素
    for (int i = 0; i < 3; i++)
    {
        ls1.push_back(i);
    }

    //删除头部/尾部元素
    ls1.pop_front();
    ls1.pop_back();

    //使用迭代器遍历
    for(auto it = ls1.begin();it!= ls1.end();it++)
    {
        std::cout << *it << std::endl;
    }

    return 0;
}

```

## 5.string字符串

#### 特点
- 可以很方便地处理各种字符串，比char数组方便一万倍。

#### 缺陷
- 跟vector一个性质，所以也继承了vector的缺点，插入数据不是那么快

### 代码演示

``` c++

#include <iostream>
#include <string>

int main()
{
    std::string s = "";

    //可以跟变量一样进行加减
    s = s + "ACE";
    s = s + ",NB!";

    //可以搜索字符串中的位置，返回下标首个位置，搜索不到会返回string::npos
    auto pos = s.find("ACE");
    auto pos_faild = s.find("hello");

    if(pos != std::string::npos)
    {
        std::cout << "里头有'ACE'" << std::endl;
    }

    if(pos_faild == std::string::npos)
    {
        std::cout << "找不到字符串" << std::endl;
    }

    //中文占用1个空字符,从0开始,不传递第二个参数则输出后面全部内容
    cout << s.substr(3) << endl;		


    return 0;
}

```

## 6.stack栈

#### 特点
- 和队列相反，采用 **“先进后出”** 的结构，想象成一个桶，你往里面丢东西叫做压栈，取东西的时候只能从上面取，只能拿顶部的东西，叫做出栈。

#### 缺陷
- 只能从栈顶操作，要知道底下的东西就要依次出栈。

### 代码演示

``` c++

#include <iostream>
#include <stack>

int main()
{
	//创建栈
	std::stack<int> s;
	
	//压栈
	s.push(2);
	s.push(1);
	s.push(3);

	std::cout << s.top() << std::endl;	//输出栈顶,后进后出

	//出栈
	s.pop();	//把3取出来

	s.push(3);
	
	//访问栈顶
	std::cout << s.top() << std::endl;

    //不允许使用迭代器，需要使用for循环依次访问栈顶，输出栈顶，出栈
	for (int i = 1; i <= count; i++)
	{
		std::cout << s.top() << std::endl;
		s.pop();
	}

	s.push(2);
	s.push(1);
	s.push(3);

	//获取长度
	std::cout << "栈的长度是: " << s.size() << std::endl;

	return 0;
}

```

## 7.set集合

#### 特点
- 和高中学的一样，一个集合里面的所有元素都是**唯一**的，不可能有重复的元素。
- 插入数据后会自己**从小到大**自动排序好所有元素，如果是字符串则按照字典序。

#### 缺点
- 只能从小到大排序，有时候不太方便，不能存储重复元素。
- 输入后会打乱元素位置，不适合用来做位置比较之类的。

### 代码演示

``` c++

#include <iostream>
#include <set>

int set_in()
{
    std::set<int> s; // 不需要带任何参数

    // 处理集合,插入数据,自动由小到大
    s.insert(2);
    s.insert(1);
    s.insert(3);

    for (auto i = s.begin(); i != s.end(); i++) // 遍历依旧使用迭代器
    {
        std::cout << *i << " ";
    }
    std::cout << std::endl;

    // 查找集合内元素
    // find，返回布尔值
    std::cout << (s.find(2) != s.end()) << std::endl; 
    // 这里s.find(5)与s.end()返回值一致，所以判断式不成立，返回0
    std::cout << (s.find(5) != s.end()) << std::endl;

    s.erase(2);                                       // 擦除2
    std::cout << (s.find(2) != s.end()) << std::endl; // 返回0

    return 0;
}

```

## 8.map键值对

#### 特点
- 两个元素之间形成映射的关系，可以把一个类型映射到一个值。
- 自动按照ASCII码从小到大排序,首字母相同比较下一位,同时键x唯一，后面覆盖前面的。

#### 缺陷
- 一个键只能对应一个值，同时牺牲很多空间，因为底层是红黑树的结构

### 代码演示

``` c++

#include <map>
#include <iostream>

int main()
{
	std::map <std::string, int> m;

	//添加
	m["Hello"]  = 2;	//类似映射，把字符串映射到整形
	m["World"] = 3;

	std::cout << m["Hello"] << std::endl;

    //可以使用迭代器遍历
	for (auto i = m.begin(); i != m.end(); i++)
	{
		std::cout << i->first << " 是 " << i->second << std::endl;
	}

	//获取长度
	std::cout << m.size() << std::endl;

	return 0;
}

```
---
title: c++类和对象编程范式学习的笔记
published: 2025-11-24
pinned: false
description: 个人总结的一些c++类和对象编程的学习笔记
tags: [coding, bloging,cpp]
category: 代码
draft: false
date: 2025-11-24
pubDate: 2025-11-24
---

# 面向对象编程笔记
## 为什么要使用类来编程
  以下面的演示代码为例子，我们需要登记学生的语文数学英语成绩,采用结构体进行存储。
  ``` c++

#include <iostream>
  struct student
  {
    int chinese;
    int math;
    int english;
    int total;
  };

  int main()
  {
    student s;
    
    std::cout << "请输入语文成绩: ";
    std::cin >> s.chinese;
    std::cout << "请输入数学成绩: ";
    std::cin >> s.math;
    std::cout << "请输入英语成绩: ";
    std::cin >> s.english;
    
    s.total = s.chinese + s.math + s.english;
    std::cout << "总成绩: " << s.total << std::endl;
    return 0;
  }
```
**1.cpp**
如上面所示，我们使用一个结构体来存储语数英和总分成绩，但是main主函数一般是用来存储一些主要的任务，我们也可以单独把上面的函数拿出来，在c++里面，为了方便管理，采用把一些数据和使用的函数封装起来，变成了类(class)，下面是使用类来实现语数英成绩的登记。

``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;
    int english;
    int total;
    int count;

    student(int chinese = 0, int maths = 0, int english = 0, 
    int total = 0, int count = 0)
    {
        this->chinese = chinese;
        this->maths = maths;
        this->english = english;
        this->total = total;
        this->count = count;
    }

    void set()
    {
        std::cout << "请输入";
        switch (count)
        {
        case (0):
            std::cout << "语文成绩:";
            std::cin >> chinese;
            break;
        case (1):
            std::cout << "数学成绩:";
            std::cin >> maths;
            break;
        case (2):
            std::cout << "英语成绩:";
            std::cin >> english;
            break;
        }
        count++;
        total = chinese + maths + english;
    }

    void show()
    {
        std::cout << "总成绩: " << total << std::endl;
    }
};

int main()
{
    student stu;
    for (int i = 0; i < 3; i++)
    {
        stu.set();
    }
    stu.show();
    return 0;
}
```
**2.cpp**
上面的功能使用类进行了实现，可以看到所有的功能都被封装进了类里面，主函数的功能更加清晰明了，增加了程序的解耦性，接下来开始记录下类的特色。

### 类的访问权限
c++类的一大特色就是封装性，可以把一些接口与数据隐蔽起来，只暴露一些接口出来，为了方便划分，分了以下三类
- **puclic:**公有成员，允许在类的外部进行访问。
- **private:**私有成员，仅允许在类的里面进行访问。
- **protected:**保护成员，允许在类里面和派生类进行数据访问。

### 类的构造函数和析构函数
c++中的类默认会自动调用构造函数，并且在对象完成(销毁的时候)自动运行析构函数，当然也可以手动写这两个函数来实现功能，或者自定义一些函数下去。

#### 构造函数：
当它主动写出来的时候，不会执行类自带的默认构造函数，而是会执行构造函数的内容，如果带有参数的时候，可以允许创建类的时候一同初始化，**函数有特定的格式，没有返回值，会取缔原来的默认构造函数**下面是演示代码，还是登记语数英成绩。
**构造函数可以存在多个，你在类里面传递不同的值会自动帮你匹配不同的构造函数**

``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;
    int english;
    int total;
    
    //默认初始化的构造函数
    student() {}

    //有三个值进入的构造函数
    student(int a,int b,int c)
    {
        chinese = a;
        maths = b;
        english = c;
        total = a + b + c;
        std::cout << "已进入了自定义的构造函数" << std::endl;
    }

    //有两个值进入的构造函数
    student(int a,int b):chinese(a),maths(b)
    {
        std::cout << "已进入了自定义的构造函数" << std::endl;
    }

    void show()
    {
        std::cout << "总成绩: " << total << std::endl;
    }
};

int main()
{
    int chinese;
    int maths;
    int english;
    
    std::cout << "请输入语数英成绩(空格隔开):";
    std::cin >> chinese >> maths >> english;
    //下面使用三个参数，就会匹配上面第二个构造函数
    student stu1(chinese,maths,english);

    stu1.show();
    
    return 0;
}
```
**3.cpp**

#### 析构函数
一般来说，在类的生命周期结束的时候，会自动执行析构函数销毁，当然也可以手动执行。
**一个类里面只允许一个析构函数，一般来说只有使用了动态分配的时候需要手动写一个析构函数**
还是以成绩为例子，使用数组存储n个学生的数据。

``` c++
#include <iostream>

class student
{
public:
    int *score;
    int size;

    student(int s)
    {
        size = s;
        score = new int[size];
    }

    void write()
    {
        for (int i = 0; i < size; i++)
        {
            std::cin >> score[i];
        }
    }

    void show()
    {
        for (int i = 0; i < size; i++)
        {
            std::cout << "第" << i << "个学生的成绩是" << score[i] << std::endl;
        }
    }

    ~student()
    {
        delete[] score;
        std::cout << "score数组已经被清理!" << std::endl;
    }
};

int main()
{
    int n = 0;
    std::cout << "请输入成绩的数量";
    std::cin >> n;
    student stu(n);

    stu.write();
    stu.show();

    return 0;
}
```
**4.cpp**

### 拷贝构造函数
如果在写代码的时候需要使用已存在的对象新建一个对象，就会调用到拷贝构造函数而不是普通的构造函数，如果没有写，编译器会自己提供，用来把原来成员的值赋值给新的成员变量,先来看下面第一份代码。

``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;

    student(int a, int b) : chinese(a), maths(b)
    {
        std::cout << "调用了一次构造函数" << std::endl;
    }
};

int main()
{
    int a = 91;
    int b = 91;
    student stu1(a, b);

    //第一种写法，括号里面包含原来的对象
    student stu2(stu1);

    //第二种写法，使用赋值符号
    student stu3 = stu1;


    return 0;
}
```
**5.cpp**
运行之后查看终端，可以发现，只打印了一行“调用了一次构造函数”，说明后面的拷贝已经不再使用原来自带的构造函数了。

接下来介绍拷贝构造函数的语法，定义如下:
**类名(const 类名& 新的对象名,一些参数) {...},其中参数可以有也可以没有**,下面是代码演示，录入原来的成绩，新的成绩少十分。

``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;

    student(int a, int b) : chinese(a), maths(b)
    {
        std::cout << "调用了一次构造函数" << std::endl;
    }

    student(const student &other, int num)
    {
        std::cout << "调用了一次拷贝构造函数" << std::endl;
        chinese = other.chinese - num;
        maths = other.maths - num;
    }

    student(const student &other)
    {
        chinese = other.chinese;
        maths = other.maths;
        std::cout << "调用了一次拷贝构造函数，但是没有-10" << std::endl;
    }

    void show()
    {
        std::cout << chinese << ' ' << maths << std::endl;
    }
};

int main()
{
    int a = 91;
    int b = 91;
    int n = 10;

    student stu1(a, b);
    stu1.show();

    student stu2(stu1);
    stu2.show();

    student stu3(stu2, n);
    stu3.show();

    return 0;
}
```
**6.cpp**
### 深拷贝&&浅拷贝
举个例子，有这么一个指针，指向地址a，使用拷贝构造函数拷贝对象b，对象b的指针也被拷贝，也是指向地址a，假如在对象b中对指针进行操作并且释放了，那指针就变成了野指针，这时候再回到对象a再调用这个指针，程序就会爆炸，为了避免这个问题，就要执行深拷贝，同时防止其他对象操作指针影响到原来的值，下面的代码演示了浅拷贝下变量和指针指向的地址。
``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;
    int *ptr; // 这里多了一个新的指针ptr

    student(int a, int b) : chinese(a), maths(b)
    {
        ptr = new int(114514);
        std::cout << "调用了一次构造函数" << std::endl;
    }

    student(const student &other)
    {
        chinese = other.chinese;
        maths = other.maths;
        ptr = other.ptr;
        std::cout << "调用了一次拷贝构造函数，但是没有-10" << std::endl;
    }

    void show()
    {
        std::cout << chinese << ' ' << maths << std::endl;
        std::cout << "指针地址是";
        std::cout << ptr << std::endl;
        std::cout << "指针内容是" << *ptr << std::endl;
    }

    void showptr()
    {
        std::cout << "指针内容是" << *ptr << std::endl;
    }

    ~student()
    {
        delete ptr;
        ptr = nullptr;
    }
};

int main()
{
    int a = 91;
    int b = 91;

    student stu1(a,b);
    stu1.show();

    student stu2(stu1);
    stu2.show();
    *stu2.ptr = 666;
    stu1.showptr();
    stu2.showptr();

    return 0;
}
```
**7.cpp**
入上面所示，新的指针和旧的指针都是相同的地址,所以使用析构函数就会出现问题，终端内容运行如下
```
调用了一次构造函数
91 91
指针地址是0x1003044a0
指针内容是114514
调用了一次拷贝构造函数，但是没有-10
91 91
指针地址是0x1003044a0
指针内容是114514
指针内容是666
指针内容是666
7(6055,0x100090600) malloc: *** error for object 0x1003044a0: pointer being freed was not allocated
7(6055,0x100090600) malloc: *** set a breakpoint in malloc_error_break to debug
```
**终端内容**
错误提示不能重复释放不存在的区域，同时我修改了stu2的内容，却能影响stu1，为此需要使用深拷贝来解决这一问题，以下是语法:
**ptr = new int;**
**memcpy(ptr, other.ptr, sizeof(int));**
``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;
    int *ptr; // 这里多了一个新的指针ptr

    student(int a, int b) : chinese(a), maths(b)
    {
        ptr = new int(114514);
        std::cout << "调用了一次构造函数" << std::endl;
    }

    student(const student &other)
    {
        chinese = other.chinese;
        maths = other.maths;
        ptr = new int(*other.ptr);
        std::cout << "调用了一次拷贝构造函数，但是没有-10" << std::endl;
    }

    void show()
    {
        std::cout << chinese << ' ' << maths << std::endl;
        std::cout << "指针地址是";
        std::cout << ptr << std::endl;
        std::cout << "指针内容是" << *ptr << std::endl;
    }

    void showptr()
    {
        std::cout << "指针内容是" << *ptr << std::endl;
    }

    ~student()
    {
        delete ptr;
        ptr = nullptr;
    }
};

int main()
{
    int a = 91;
    int b = 91;

    student stu1(a,b);
    stu1.show();

    student stu2(stu1);
    stu2.show();
    *stu2.ptr = 666;
    stu1.showptr();
    stu2.showptr();

    return 0;
}
```
**8.cpp**
```
调用了一次构造函数
91 91
指针地址是0x1004041c0
指针内容是114514
调用了一次拷贝构造函数，但是没有-10
91 91
指针地址是0x1004041d0
指针内容是114514
指针内容是114514
指针内容是666
```
**终端内容**
可以看到上面的内容正常运行而且出现了两个114514，指针地址也不一样了，说明已经是独立的内存区域了。

### 初始化列表
在创建新的类时候，除了原来的拷贝构造函数写法，还有另外一种写法,它的语法是:
**类名(参数列表):成员1(参数1),成员2(参数2)...成员n(参数n)**

### 继承
允许在一个类复制另一个类已有的成员和方法的同时增加自己的方法，使用冒号:来进行继承,以下是代码演示。
``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;

    student(int a, int b) : chinese(a), maths(b) {}

    void show()
    {
        std::cout << chinese << " " << maths << std::endl;
    }
};

class stu2 : public student
{
public:
    int english = 88;

    stu2(int a, int b) : student(a, b) {}

    void show_all()
    {
        std::cout << chinese << " " << maths << " " << english << std::endl;
    }
};

int main()
{
    int chinese = 91;
    int maths = 91;
    student stu(chinese, maths);
    stu.show();

    stu2 stu3(85, 90);
    stu3.show_all();
    stu3.show();

    return 0;
}
```
**9.cpp**
如上所示，stu2中并没有声明show，chinese和maths,但是一样可以正常输出，说明原来的类中public的组员已经被继承过来，变成了stu2的成员。

### 多态
多态中允许一个函数以新的方式重写之前旧的函数，使用virtual(虚类)关键字来进行，下面是演示代码。
``` c++
#include <iostream>

class student
{
public:
    int chinese;
    int maths;

    virtual void show()
    {
        std::cout << chinese << ' ' << maths << std::endl;
    }
};

class student1 : public student
{
public:
    int english;

    void show()
    {
        std::cout << chinese << ' ' << maths << ' ' << english << std::endl;
    }
};

int main()
{
    student stu1;
    stu1.chinese = 91;
    stu1.maths = 91;
    stu1.show();

    student1 stu2;
    stu2.chinese = 91;
    stu2.maths = 91;
    stu2.english = 91;
    stu2.show();
    
    return 0;
}
```
**10.cpp**

# C++

#### C++的内存管理
* **内存分配方式**
    在C++中，内存一般分成5个区，即堆、栈、自由存储区、全局/静态存储区和常量存储区。
    **栈 :** 函数执行过程的局部变量存储空间。
    **堆 :** `new`运算符操作的内存空间。
    **自由存储区 :** `malloc`函数操作的内存空间。
    **全局/静态存储区 :** 全局变量和静态变量存储空间。
    **常量存储区 :** 常量的存储空间
* **内存泄露及解决办法**
    程序分配的内存未能释放；内存泄漏并非物理内存消失，而是失去分配内存的控制。
    1. `new/malloc`申请资源后没有使用`delete/free`释放。
    2. 拷贝/赋值构造过程中进行`浅拷贝`。
    3. 有子类继承的父类析构函数没有定义为`虚函数`。

    **解决办法 :** 通过`对象`管理资源；使用`智能指针`。

#### 堆和栈的区别
* **空间分配机制不同 :** 栈是由`编译器`自动分配和释放；堆是程序员通过`new`和`delete`进行空间分配和释放的。
* **缓存方式不同 :** 栈使用的是`一级缓存`, 生命周期结束立即释放；堆使用`二级缓存`, 速度会慢些。

* **数据结构不同 :** 栈即栈结构，先进后出；堆类似数组结构。
* **地址生长方向不同 :** 堆是`向上增长`，低地址向高地址；栈是`向下增长`，高地址向低地址。

#### new和malloc的区别
* `new`是运算符，可以重载；申请内存无需显式指定内存大小，内存分配失败抛出`bac_alloc`异常；会自动调用类的构造函数和析构函数；
* `malloc`是标准库函数，申请内存需要显式指定内存大小，内存分配失败返回`nullptr`。

#### 智能指针和普通指针的区别
智能指针本质是`template class`，秉行以对象管理资源方式，使得资源通过类的构造函数获取和析构函数释放，避免内存泄漏。

**智能指针 `defined in the header <memory>`**
* `unique_ptr`实现严格拥有概念，**独占所有权**，保证同一时间只有一个智能指针指向该对象。**该指针不支持拷贝/赋值构造，但可以移动构造/赋值。**
* `shared_ptr`实现共享式拥有概念，**共享所有权**，允许多个智能指针指向该对象。实现机制是**拷贝构造时使用同一份引用计数。**
* `weak_ptr`辅助智能指针，**弱共享所有权**，主要为了解决`shared_ptr`中循环引用导致引用计数始终不为0而造成内存泄漏。实现机制是**只是单纯访问对象，构造和析构时都不会改变引用计数。**

    ```
    // C++11中 is deprecated, C++17中 is removed.
    template <class T>
    class  auto_ptr {
    private:
        T *p{nullptr};
    public:
        auto_ptr(T *s) : p{s} {}
        ~auto_ptr() {delete p;}

        auto_ptr(auto_ptr &a) {
            p = a.p;
            a.p = nullptr;
        }

        auto_ptr &operator=(auto_ptr &a) {
            delete p;
            p = a.p;
            a.p = nullptr;
            return *this;
        }

        T &operator*() const {return *p;}
        T *operator->() const {return *p;}
    };
    ```
    ```
    template <class T>
    class unique_ptr {
    private:
        T *p;
    public:
        unique_ptr() :p{} {}
        unique_ptr(T *s) :p{s} {}
        ~unique_ptr() {delete p;}

        unique_ptr(unique_ptr &&s) :p{s.p} {s.p = nullptr;}
        unique_ptr &operator=(unique_ptr s) {
            delete p;
            p = s.p;
            s.p = nullptr;
            return *this;
        }

        T *operator->() const {return p;}
        T &operator*() const {return *p;}

    private:
        unique_ptr(const unique_ptr &) = delete;
        unique_ptr &operator=(const unique_ptr &) = delete;
    };
    ```
    ```
    template <class T>
    class weak_ptr;
    class Counter {
    public:
        Counter() = default;
        int s_ = 0; // shared_ptr的计数
        int w_ = 0; // weak_ptr的计数
    };

    template <class T>
    class shared_ptr {
    private:
        T *ptr_;
        Counter *cnt_;
    public:
        shared_ptr(T *p = 0) :ptr_{p} {
            cnt_ = new Counter();
            if(p) {
                cnt_->s_ = 1;
            }
        }

        ~shared_ptr() {
            release();
        }

        shared_ptr(shared_ptr<T> const &s) {
            ptr_ = s.ptr_;
            (s.cnt)->s_++;
            cnt_ = s.cnt_;
        }

        shared_ptr(weak_ptr<T> const &w) {
            ptr_ = w.ptr_;
            (w.cnt_)->s++;
            cnt_ = w.cnt_;
        }

        shared_ptr<T> &operator=(shared_ptr<T> &s) {
            if(this != &s) {
                release();
                (s.cnt_)->s++;
                cnt_ = s.cnt_;
                ptr_ = s.ptr_;
            }
            return *this;
        }

        T &operator*() {
            return *ptr_;
        }

        T *operator->() {
            return ptr_;
        }

        friend class weak_ptr<T>;

    protected:
        void release() {
            cnt_->s--;
            if(cnt_->s_ < 1) {
                delete ptr_;
                if(cnt_->w_ < 1){
                    delete cnt_;
                    cnt_ = nullptr;
                }
            }
        }
    }
    ```
    ```
    template <class T> {
    class weak_ptr {
    private:
        T *ptr_{nullptr};
        Counter *cnt_{nullptr};
    public:
        weak_ptr() = default;
    
        weak_ptr(shared_ptr<T> &s) :ptr_{s.ptr_}, cnt(s.cnt_) {
            cnt<->w_++;
        }
        
        weak_ptr(weak_ptr<T> &w) :ptr_{w.ptr_}, cnt{w.cnt_} {
            cnt_->w++;
        }

        ~weak_ptr() {
            release();
        }

        weak_ptr<T> &operator=(shared_ptr<T> &s) {
            release();
            cnt_ = s.cnt_;
            cnt_->w++;
            ptr_ = s.ptr_;
            return *this;
        }

        weak_ptr<T> &operator=(weak_ptr<T> &w) {
            if(this != &w) {
                release();
                cnt_ = w.cnt_;
                cnt_->w++;
                ptr_ = w.ptr_;
            }
            return *this;
        }

        shared_ptr<T> lock() {
            return shared_ptr<T>(*this);
        }

        bool expired() {
            if(cnt && cnt->s_ > 0) {
                return false;
            }
            return true;
        }

        friend class shared_ptr<T>;

    protected:
        void release() {
            if(cnt_) {
                cnt_->w--;
                if(cnt_->w_ < 1 && cnt_->s_ < 1) {
                    cnt_ = nullptr;
                }
            }
        }
    }
    ```

注：A `weak_ptr` is created as a copy of a `shared_ptr`; 无法直接通过 `weak_ptr` access to 共享 object; `weak_ptr` 提供了一个 lock 方法，始终返回一个 `shared_ptr` 实例。

#### lambada表达式
* 基本语法
    ```
    [capture](parameters) specifiers exception attr -> return type { /* code; */}
    
    [capture]: 捕获列表(外部变量)，捕获类型分为值捕获、引用捕获和隐式捕获；
    [parameters]: 参数列表；
    specifiers exception attr: 附加说明符，一般为 mutable、noexcept等；
    ->return type: lambda函数的返回类型，一般不需要；
    {}: 函数主体。
    ```
* 编译器对lambda的生成规则:
    ```
    捕获列表 = 类的 private成员；
    形参列表 = 类成员函数 operator()的形参列表；
    mutable = 类成员函数 operator()的常属性 const;
    返回类型 = 类成员函数 operator()的返回类型；
    函数体 = 类成员函数 operator()的函数体。

    ```
    

#### vector底层实现机制
`vector`底层是**动态数组**，主要通过`start`、`finish`和`end_of_storage`三个迭代器进行内存操作。
    
    size(): finish - start;
    capacity(): end_of_storage - start;
    

**vector内存增长机制 :** 当空间不够时，会自动申请1.5/2.0倍原空间大小的内存，再把原内容拷贝过来，然后才开始构造新元素并释放原空间。因此任何引起vector容器空间重新分配的操作都会使原迭代器失效。

**vector中reserve()和resize()的区别 :** `reserve()` 改变的是vector容器的`capacity`；`resize()` 改变的是vector容器的`size`。




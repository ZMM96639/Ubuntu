//#include <iostream>
//#include <memory>
//using namespace std;
//class A
//{
//public:
//    int i;
//    A(int n):i(n) { };
//    ~A() { cout << i << " " << "destructed" << endl; }
//};
//int main()
//{
//    shared_ptr<A> sp1(new A(2)); //A(2)由sp1托管，
//    shared_ptr<A> sp2(sp1);       //A(2)同时交由sp2托管
//    shared_ptr<A> sp3;
//    sp3 = sp2;   //A(2)同时交由sp3托管
//    cout << sp1->i << "," << sp2->i <<"," << sp3->i << endl;
//    A * p = sp3.get();      // get返回托管的指针，p 指向 A(2)
//    cout << p->i << endl;  //输出 2
//    sp1.reset(new A(3));    // reset导致托管新的指针, 此时sp1托管A(3)
//    sp2.reset(new A(4));    // sp2托管A(4)
//    cout << sp1->i << endl; //输出 3
//    sp3.reset(new A(5));    // sp3托管A(5),A(2)无人托管，被delete
//    cout << "end" << endl;
//    return 0;
//}

#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <mutex>

struct Test
{
    Test() { std::cout << "  Test::Test()\n"; }
    ~Test() { std::cout << "  Test::~Test()\n"; }
};

//线程函数
void thr(std::shared_ptr<Test> p)
{
    //线程暂停1s
    std::this_thread::sleep_for(std::chrono::seconds(1));

    //赋值操作, shared_ptr引用计数use_cont加1(c++11中是原子操作)
    std::shared_ptr<Test> lp = p;
    {
        //static变量(单例模式),多线程同步用
        static std::mutex io_mutex;

        //std::lock_guard加锁
        std::lock_guard<std::mutex> lk(io_mutex);
        std::cout << "local pointer in a thread:\n"
                  << "  lp.get() = " << lp.get()
                  << ", lp.use_count() = " << lp.use_count() << '\n';
    }
}

int main()
{
    //使用make_shared一次分配好需要内存
    std::shared_ptr<Test> p = std::make_shared<Test>();
    //std::shared_ptr<Test> p(new Test);

    std::cout << "Created a shared Test\n"
              << "  p.get() = " << p.get()
              << ", p.use_count() = " << p.use_count() << '\n';

    //创建三个线程,t1,t2,t3
    //形参作为拷贝, 引用计数也会加1
    std::thread t1(thr, p), t2(thr, p), t3(thr, p);
    std::cout << "Shared ownership between 3 threads and released\n"
              << "ownership from main:\n"
              << "  p.get() = " << p.get()
              << ", p.use_count() = " << p.use_count() << '\n';
    //等待结束
    t1.join(); t2.join(); t3.join();
    std::cout << "All threads completed, the last one deleted\n";

    return 0;
}
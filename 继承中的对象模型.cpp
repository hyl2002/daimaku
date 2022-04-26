#include<iostream>
using namespace std;

class Person{
public:
   int m_a;
protected:
   int m_b;
private:
   int m_c;

};
class son:public Person{
public:
   int m_d;
};
void test01(){
cout<<"继承是如何进行继承的？         "<<"输出继承的类的大小"<<sizeof(son)<<endl;//输出的结果是16
//这说明了子类继承父类的话，不管这个类的类型是公共的还是私有的，它全部都继承了，只不过继承的时候将父类中的私有成员进行了编译器的隐藏，素以无法进行访问但还是尽心了继承操作的；

}
int main(){
test01();



return 0;    
}
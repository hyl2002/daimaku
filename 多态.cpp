#include<iostream>
using namespace std;

//多态的使用
class Animal{
public:
   void  virtual speak(){
       cout<<"动物在吼叫！！"<<endl;
   }
};
class cat:public Animal{
public:
    void speak(){
        cout<<"小猫在吼叫！！"<<endl;
    }  

};
void shuispeak(Animal &animal){
   animal.speak();
}
void test01(){
    cat t;
    shuispeak(t);//此时得出来的结果时动物在吼叫，因为这个是静态的多态，在编译阶段的地址就被绑定了
    //如何将静态的多态转化为动态的多态，只需要将父类的函数前加上virtual就好
}
int main(){
test01();
return 0;
}
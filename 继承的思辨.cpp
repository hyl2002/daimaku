#include<iostream>
using namespace std;

class Person{
public:
  Person(string name,int age,int gend)
  {
    this->m_age=age;
    this->m_name=name;
    this->gender=gend;
  }
string getname(){
    cout<<"年龄是："<<this->getage()<<endl;
    return this->m_name;
}
private:
  int getage()
  {   
      return this->m_age;
  }
public:
  string m_name;
  int m_age;
private:
  int gender;
};

class son:public Person{
public:
son(string name,int age,int gende):Person(name,age,gende){}
//如果私有属性的话，也是可以通过一些公共的接口函数来进行访问
//初始化列表中，前者要和后者保持一直，加油奥里给
};
void test01()
{
son s("li",10,1);
cout<<s.m_age<<endl;
// cout<<s.m_name;
cout<<"姓名是："<<s.getname();

}
int main()
{
    test01();
system("pause");
return 0;
}
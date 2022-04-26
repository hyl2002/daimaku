#include<iostream>
#include<string>
using namespace std;
#include<fstream>


char buff[100];
bool flag=true;
int count;
string b[100];
string s;
void test01(){
  ifstream inf;
  inf.open("h.txt",ios::in);
  if(!inf.is_open())
  {
      cout<<"文件读取失败!";
  }
  else{
      while(inf>>buff)
      {
      cout<<buff<<endl;
      b[count]=buff;
      count++;
      }cout<<"-----------------"<<endl;
  }
   for(int i=0;i<10;i++)
 {
     cout<<b[i]<<endl;
 }
 }

int main(){
test01();
system("pause");
return 0;
}
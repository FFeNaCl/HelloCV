#include<iostream>
#include<cstring>
#include<fstream>
using namespace std;
int c1,c2,c3;string fin,s1,s2,s3;int a=0;//c1选择算法c2初始密钥s1初始文本 
string Crypto(int ru)
{
        int flag;string chu,key;
        if(ru==1)//凯撒 
        {
            if(a==2||a==4)
            {
                c2=-c2;
            }
            for(int i=0;i<=s1.length()-1;i++)
            {
                while(1)
                {
                    if(('a'<=s1[i]&&s1[i]<='z')||('A'<=s1[i]&&s1[i]<='Z'))//如果是字母 
                    {
                        if(c2>=26)//正 
                        {
                            c2-=26;
                            continue;
                        }
                        if(c2>0)//正 
                        {
                            if('a'<=s1[i]&&s1[i]<='z')
                            {
                            if((s1[i]+c2)>'z')
                            {
                                chu+=s1[i]+c2-26;
                                break;
                            }
                            else
                            {
                                chu+=s1[i]+c2;
                                break;
                            }
                            }
                            if('A'<=s1[i]&&s1[i]<='Z')
                            {
                            if((s1[i]+c2)>'Z')
                            {
                                chu+=s1[i]+c2-26;
                                break;
                            }
                            else
                            {
                                chu+=s1[i]+c2;
                                break;
                            }
                            }
                        }
                        
                        if(c2<=-26)//负 
                        {
                            c2+=26;
                            continue;
                        }
                        if(c2<=0)//负 
                        {
                            if('a'<=s1[i]&&s1[i]<='z')
                        {
                            if((s1[i]+c2)<'a')
                            {
                                chu+=s1[i]+c2+26;
                                break;
                            }
                            else
                            {
                                chu+=s1[i]+c2;
                                break;
                            }
                        }
                        if('A'<=s1[i]&&s1[i]<='Z')
                        {
                            if((s1[i]+c2)<'A')
                            {
                                chu+=s1[i]+c2+26;
                                break;
                            }
                            else
                            {
                                chu+=s1[i]+c2;
                                break;
                            }
                        }
                        }    
                    }//字母 
                    else     //数字 
                        {chu+=s1[i];break;}
                }
            }
            return chu;
        } 
        if(ru==2)//xor
        {
            string key=to_string(c2);
            int key1=key.length();
            for (int i=0;i<s1.length();++i) 
            {
                s1[i]=s1[i]^key[i%key1];
            }
            return s1;
        }    
}
void FileHandler(string filename) //这里用了点AI 
{
        string s4;
        ifstream file(filename);
        string content((istreambuf_iterator<char>(file)), {});
        s1=content;
        s4=Crypto(c1);
        cout<<"请输入保存文件名：";
        cin>>fin;
        ofstream fout(fin);
        fout<<s4;
        cout<<s4<<endl; 
        fout.close();
}
void Menu(int a)
{
    if(a==0)
    {
        cout<<"请输入你需要的模式："<<endl<<"文本加密--1  文本解密--2  用文件加密--3  用文件解密--4"<<endl; 
    }
    if(a==1)
    {
        cout<<"请选择加密算法：凯撒--1，XOR--2"<<endl;
        cin>>c1;
        cout<<"请输入加密文本：";
        cin>>s1;
        cout<<"请输入密钥：";
        cin>>c2;
        fin=Crypto(c1);
        cout<<"加密结果：";
        cout<<fin<<endl;
        s2=fin;
    }
    if(a==2)
    {
        cout<<"请选择解密算法：凯撒--1，XOR--2"<<endl;
        cin>>c1;
        cout<<"请输入解密文本：";
        cin>>s1;
        cout<<"请输入密钥：";
        cin>>c2;
        fin=Crypto(c1);
        cout<<"解密结果：";
        cout<<fin<<endl;
        s2=fin;
    }
    if(a==3)
    {
        cout<<"请输入加密文档路径：";
        cin>>s1;
        cout<<"请选择加密算法：凯撒--1，XOR--2"<<endl;
        cin>>c1;
        cout<<"请输入密钥：";
        cin>>c2;
        FileHandler(s1); 
        cout<<"加密结果已保存到：";
        cout<<fin;
    }
    if(a==4)
    {
        cout<<"请输入解密文档路径：";
        cin>>s1;
        cout<<"请选择解密算法：凯撒--1，XOR--2"<<endl;
        cin>>c1;
        cout<<"请输入密钥：";
        cin>>c2;
        FileHandler(s1); 
        cout<<"解密结果已保存到：";
        cout<<fin;
    }
}
int main() 
{
    Menu(a);cin>>a;Menu(a);
    return 0;
}
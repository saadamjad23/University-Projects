#include<iostream>
#include<string>

using namespace std;

struct registry
{
	string first_name[20];
	string last_name;
	string gender;
	string contact_no;
	string address;
	string gmail;
	string password1;
	string password2;
};

main()
{
	registry r;
	for(int i=0;i<120;i++)
	cin>>r.first_name[i];
	cout<<r.first_name[20];
	return 0;
}


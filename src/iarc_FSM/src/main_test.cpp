#include <iostream>
#include "FSM.h"
#include <vector>

using namespace std;

int main(int argc, char const *argv[])
{
	vector<float> pos(3);
	FSM test;

	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j <3; ++j)
		{
			cin >> pos[j];
		}
		test.changeAction(pos);
		test.changeStatus();
		cout << test.getStatus() << "\t" << test.getAngle() << endl;
	}
	
	return 0;
}
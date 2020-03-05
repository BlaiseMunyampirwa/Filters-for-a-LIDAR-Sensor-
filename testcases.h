/* Testing Range and Temporal filter for LIDAR generated scans
 * Blaise Munyampirwa
 * University of Chicago. March, 2020
 */


#include<string>

class testcases{
private:
	string name;
public:
	vector<vector<float> > data;
	vector<float> scan;
	vector<float> med_out;
	vector<float> range_out;
	testcases(string name):name(name){}
	string get_name(){
		return name;
	}
	void set_name(string input){
		name = input;
	}
	~testcases(){}
};

//Structure containing different functions for testing pursposes
struct Test{
	//test the correctness of filtered range and temporal median
	void check_FD(filter &L, testcases &T); 
	//test the correctness of large temporal scans
	void check_temp(filter &L, unsigned int D); 
	//create test-cases
	vector<testcases> Build_FD(); 
	void print(vector<float> &scan); 
	//display the filtered output of temporal scans
	void rand_update(filter &L, unsigned int N, unsigned int D); 
	void tests();
};

void Test::check_FD(filter &L, testcases &T){
	if (L.update_median(T.data) == T.med_out &&
		L.update_range(T.scan) == T.range_out){
		cout << T.get_name() << ": PASSED \n";
	}
	else
	{
		cerr << T.get_name() << ": FAILED \n";
	}
}

void Test::check_temp(filter &L, unsigned int D){
	testcases* temp = new testcases(" ");
	temp->scan.resize(1e3, 0.1);
	temp->med_out = temp->scan;
	temp->range_out = temp->scan;
	for (unsigned int i = 0; i < D; i++){
		L.add_scan(temp->scan, temp->data);
		string str = "1000 measurements T";
		string index = to_string(i);
		str += index;
		temp->set_name(str);
		if (i == D - 1) check_FD(L, *temp);
	}
	delete temp;
}

vector<testcases> Test::Build_FD(){
	filter L;
	vector<testcases> FD_scans;
	testcases triv0("Empty case");
	triv0.scan = {};
	triv0.med_out = {};
	triv0.range_out = {};
	L.add_scan(triv0.scan, triv0.data);

	testcases triv1("1e30 reading values");
	triv1.scan = {-1e30, 60, 1e30};
	triv1.med_out = {-1e30, 60, 1e30};
	triv1.range_out = {0.03, 50, 50};
	L.add_scan(triv1.scan, triv1.data);

	testcases triv2("1000 measurements per scan");
	triv2.scan.resize(1e3, 0.1);
	triv2.med_out = triv2.scan;
	triv2.range_out = triv2.scan;
	L.add_scan(triv2.scan, triv2.data);

	FD_scans.push_back(triv0);
	FD_scans.push_back(triv1);
	FD_scans.push_back(triv2);

	testcases T0("Problem example T0 update");
	T0.scan = {0., 1., 2., 1., 3.};
	T0.med_out = {0, 1., 2., 1., 3.};
	T0.range_out = {0.03, 1., 2., 1., 3.};

	testcases T1("Problem example T1 update");
	T1.scan = {1., 5., 7., 1., 3.};
	T1.med_out = {0.5, 3., 4.5, 1., 3.};
	T1.range_out = {1., 5., 7., 1., 3.};

	testcases T2("Problem example T2 update");
	T2.scan = {2., 3., 4., 1., 0.};
	T2.med_out = {1., 3., 4., 1., 3.};
	T2.range_out = {2., 3., 4., 1., 0.03};

	testcases T3("Problem example T3 update");
	T3.scan = {3., 3., 3., 1., 3.};
	T3.med_out = {1.5, 3., 3.5, 1., 3.};
	T3.range_out = {3., 3., 3., 1., 3.};

	testcases T4("Problem example T4 update");
	T4.scan = {10., 2., 4., 0., 0.};
	T4.med_out = {2., 3., 4., 1., 3.};
	T4.range_out = {10., 2., 4., 0.03, 0.03};

	for (unsigned int step = 0; step < 5; step++){
		if (step == 0) {
			L.add_scan(T0.scan, T0.data);
			FD_scans.push_back(T0);
		}
		else if (step == 1) {
			L.add_scan(T1.scan, T0.data);
			T1.data = T0.data;
			FD_scans.push_back(T1);
		}
		else if (step == 2) {
			L.add_scan(T2.scan, T1.data);
			T2.data = T1.data;
			FD_scans.push_back(T2);
		}
		else if (step == 3) {
			L.add_scan(T3.scan, T2.data);
			T3.data = T2.data;
			FD_scans.push_back(T3);
		}
		else if (step == 4) {
			L.add_scan(T4.scan, T3.data);
			T4.data = T3.data;
			FD_scans.push_back(T4);
		}
	}
	return FD_scans;
}

void Test::print(vector<float> &scan){
	for (auto i: scan) cout << i << "\t";
	cout << endl;
	return;
}

void Test::rand_update(filter &L, unsigned int N, unsigned int D){
	testcases* temp = new testcases(" ");
	string str = "Update filter N = ";
	str = str + to_string(N) + ", D = " + to_string(D);
	temp->set_name(str);
	cout << temp->get_name() << endl;
	for (unsigned int i = 0; i < D; i++){
		temp->scan = L.generate_scans(N);
		L.add_scan(temp->scan, temp->data);
		temp->med_out = L.update_median(temp->data);
		temp->range_out = L.update_range(temp->scan);
		cout << "T" << i << " scan: \t\t";
		print(temp->scan);
		cout << "T" << i << " Median update: \t";
		print(temp->med_out);
		cout << "T" << i << " Range update: \t";
		print(temp->range_out);
		cout << endl;
	}
	delete temp;
	return;
}
void Test::tests(){
	filter L;
	Test test;
	try {
		test.Build_FD();
	}
	catch(bad_alloc &e){
		cerr << "Error: Out of memory - Please reduce the size of the largest variable" << endl;
		return;
	}
	vector<testcases> FD_scans = test.Build_FD();
	for (unsigned int i = 0; i < FD_scans.size(); i++){
		test.check_FD(L, FD_scans[i]);
	}
	test.check_temp(L, 1000);
	test.rand_update(L, 10, 20);
	return;
}
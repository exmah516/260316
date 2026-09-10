#include "../ClampIllustration.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <iomanip>
#include <chrono>
#include <memory>
void check(bool b, const char* message = "illustration assertion failed") {
    if (!b) throw std::runtime_error(message);
}
bool near(double a, double b) { return std::abs(a-b) < 1e-9; }
int main(int argc, char** argv) {
 try {
    using clampillustration::Generator;
    auto g = std::make_unique<Generator>();
    for (int i=0;i<120;++i) g->update(i*.001,2,4,false);
    auto r=g->update(.120,12,24,true);
    check(near(r.reference_fn,2) && near(r.weight,0), "reference");
    for(int i=121;i<=200;++i) r=g->update(i*.001,12,24,true);
    check(near(r.fn,4) && near(r.ft,8) && near(r.weight,1), "full weight");
    r=g->update(.201,12,24,false);
    for(int i=202;i<=270;++i) r=g->update(i*.001,12,24,false);
    check(near(r.weight,1), "post-gate hold"); // 120 ms post-gate hold
    auto re=g->update(.271,12,24,true);
    check(near(re.reference_fn,2) && near(re.weight,1), "reentry");
    g->update(.272,12,24,false);
    for(int i=273;i<=392;++i) {
        r=g->update(i*.001,12,24,false);
    }
    g->update(.393,12,24,false);
    r=g->update(.394,12,24,false);
    check(r.weight < 1 && r.weight > .9, "fade begins"); // hold ended; fade has just begun
    for(int i=395;i<=465;++i) r=g->update(i*.001,12,24,false);
    check(near(r.fn,12) && near(r.weight,0), "fade ends");
    g->reset();
    r=g->update(0,1,2,true);
    check(!r.valid_fn && r.fn==1, "reset");
    auto a = std::make_unique<Generator>();
    auto b = std::make_unique<Generator>();
    for(int i=0;i<200;++i) {
        auto x=a->update(i*.001,i*.1,i*.2,i>=120);
        auto y=b->update(i*.001,i*.1,i*.2,i>=120);
        check(x.fn==y.fn && x.ft==y.ft, "prefix equality");
    }
    a->update(.200,999,999,false); b->update(.200,-999,-999,true);
    g->reset();
    for(int i=0;i<120;++i) g->update(i*.001,2,std::numeric_limits<double>::quiet_NaN(),false);
    r=g->update(.120,12,24,true);
    check(r.valid_fn && !r.valid_ft, "per channel");
    g->reset();
    for(int i=0;i<4;++i) g->update(i*.001,2,4,false);
    r=g->update(.004,12,24,true);
    for(int i=5;i<140;++i) r=g->update(i*.001,12,24,true);
    check(!r.valid_fn && r.fn==12, "invalid window"); // No late reference acquisition in an invalid window.
    g->reset();
    for(int i=0;i<120;++i) g->update(i*.001,2,4,false);
    r=g->update(.120,12,24,true);
    auto duplicate=g->update(.120,999,999,false);
    check(duplicate.fn==r.fn && duplicate.gate==r.gate, "duplicate");
    g->reset(); // Same reset entry point used on zero/mode/start.
    r=g->update(.121,12,24,true);
    check(!r.valid_fn && r.weight==0, "reset ref");
    auto future1 = std::make_unique<Generator>();
    auto future2 = std::make_unique<Generator>();
    std::vector<clampillustration::Result> results1,results2;
    for(int i=0;i<400;++i) results1.push_back(future1->update(i*.001,i,2*i,i>=120));
    for(int i=0;i<400;++i) results2.push_back(future2->update(i*.001,i<200?i:-999,2*i,i>=120 && i<200));
    for(int i=0;i<200;++i)
        check(results1[i].fn==results2[i].fn && results1[i].ft==results2[i].ft, "future invariance");
    std::cout << "PASS: reference, 20% deviation, transition/reentry, reset/gap, per-channel validity, causal prefix\n";
    if(argc < 3) return 0;
    std::ifstream input(argv[1]); std::ofstream out(argv[2]);
    check(bool(input) && bool(out));
    std::string line; std::getline(input,line);
    out << "t,v,moving,fixed,angle,phase,cycle,fn,ft,expected_fn,expected_ft,mode,valid,m2fn,m2ft,m2fnvalid,m2ftvalid\n";
    out << std::setprecision(17);
    g->reset(); int count=0, valid=0; double us=0;
    while(std::getline(input,line)) {
        std::vector<double> f; std::stringstream s(line); std::string v;
        while(std::getline(s,v,',')) f.push_back(std::stod(v));
        check(f.size()==17);
        const auto start=std::chrono::steady_clock::now();
        r=g->update(f[1]*1e-6,f[4],f[5],f[12]!=0);
        us+=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
        ++count; valid+=r.valid_fn;
        if(r.weight==0) check(r.fn==f[4] && r.ft==f[5]);
        if(r.weight==1 && r.valid_fn) check(near(r.fn-r.reference_fn,.2*(f[4]-r.reference_fn)));
        out << f[1]*1e-6 << ',' << f[13] << ",0,0,0," << f[3] << ',' << f[2] << ','
            << f[4] << ',' << f[5] << ',' << f[6] << ',' << f[7] << ',' << (argc>3 ? 2 : 1) << ',' << f[10]
            << ',' << r.fn << ',' << r.ft << ',' << r.valid_fn << ',' << r.valid_ft << '\n';
    }
    std::cout << "replayed="<<count<<" valid_fn="<<valid<<" average_us="<<us/count<<'\n';
 } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
 }
}

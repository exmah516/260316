#include "../ForcePulseGuard.h"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
int main(int argc,char**argv) {
    try {
        if (argc == 3) {
            std::ifstream in(argv[1]);
            std::ofstream out(argv[2]);
            check(bool(in)&&bool(out),"replay files");
            out << "sample_index,fn_counts,ft_counts,replaced,source_index,age_us,status,locked\n";
            std::string line;
            std::getline(in,line);
            forcepulse::Guard guard;
            out << std::setprecision(17);
            while (std::getline(in,line)) {
                std::istringstream row(line);
                std::string cell;
                std::vector<std::string> v;
                while(std::getline(row,cell,',')) v.push_back(cell);
                check(v.size()==4,"input columns");
                const auto r=guard.update(std::stoull(v[1]),std::stoull(v[0]),
                    std::stod(v[2]),std::stod(v[3]),true);
                out << v[0] << ',' << r.fn << ',' << r.ft << ',' << r.replaced << ','
                    << r.source_index << ',' << r.age_us << ',' << unsigned(r.status)
                    << ',' << r.locked << '\n';
            }
            return 0;
        }
        forcepulse::Guard guard;
        std::uint64_t holds=0;
        for (int i=0;i<15000;++i) {
            double fn=0,ft=0;
            int pulse=(i-1000)%2370;
            const bool event=i>=1000&&pulse>=0&&pulse<60;
            if (event) { fn=130;ft=110; }
            if (i==9500) fn=1000; // Preserve an axial-only impulse.
            const auto r=guard.update(i*1000,i,fn,ft,true);
            if (i<8110) check(!r.replaced,"first three events must pass");
            if (i>=8110 && event) {
                check(r.replaced&&r.fn==0&&r.ft==0,"locked pulse hold");
                check(r.source_index<std::uint64_t(i)&&r.age_us<=102000,"past source");
            }
            if (!event) check(!r.replaced,"normal sample preserved");
            if (i==9500) check(r.fn==1000,"axial-only impulse value");
            const auto duplicate=guard.update(i*1000,i,fn,ft,true);
            check(duplicate.replaced==r.replaced&&duplicate.source_index==r.source_index,"duplicate");
            holds+=r.replaced;
        }
        check(holds>0,"no replacement");
        const auto gap=guard.update(20000000,20000,4,5,true);
        check(!gap.replaced&&!gap.locked&&gap.status==forcepulse::Status::Reset,"gap reset");
        const auto invalid=guard.update(20001000,20001,4,5,false);
        check(!invalid.valid&&!invalid.replaced,"invalid");
        // A continuing offset must not freeze the signal indefinitely.
        guard.reset();
        for(int i=0;i<8600;++i) {
            const bool training=i>=1000&&i<8000&&(i-1000)%2370<3;
            const bool step=i>=8110;
            const auto r=guard.update(i*1000ULL,i,training||step?130:0,training||step?110:0,true);
            if(i==8211) check(!r.replaced&&!r.locked&&r.status==forcepulse::Status::Timeout,"hold timeout");
            if(i>8211) check(!r.replaced&&r.fn==130,"persistent step pass after timeout");
        }
        guard.reset();
        for(int i=0;i<1000;++i) {
            const auto r=guard.update(i*1000ULL,i,i==300?130:0,i==300?110:0,true);
            check(!r.replaced,"isolated event");
        }
        guard.reset();
        auto begin=std::chrono::steady_clock::now();
        double sum=0;
        for(int i=0;i<1000000;++i)
            sum+=guard.update(i*1000ULL,i,i%3,0,true).fn;
        const auto elapsed=std::chrono::duration<double,std::micro>(
            std::chrono::steady_clock::now()-begin).count();
        std::cout << "PASS holds=" << holds << " normal_mean_us=" << elapsed/1000000
                  << " checksum=" << sum << '\n';
        guard.reset();
        std::vector<double> times;
        times.reserve(200000);
        for(int i=0;i<200000;++i) {
            const bool event=i>=1000&&(i-1000)%2370<60;
            const auto start=std::chrono::steady_clock::now();
            const auto r=guard.update(i*1000ULL,i,event?130:0,event?110:0,true);
            times.push_back(std::chrono::duration<double,std::micro>(
                std::chrono::steady_clock::now()-start).count());
            sum+=r.fn;
        }
        std::sort(times.begin(),times.end());
        std::cout<<"MIXED p50_us="<<times[times.size()/2]<<" p99_us="<<times[times.size()*99/100]
                 <<" max_us="<<times.back()<<" checksum="<<sum<<'\n';
        return 0;
    } catch(const std::exception&e) { std::cerr<<e.what()<<'\n'; return 1; }
}

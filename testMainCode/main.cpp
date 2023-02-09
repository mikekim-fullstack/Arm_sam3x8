
#include <tuple>
#include <iostream>
#include <string>
#include <stdexcept>
#include <math.h>
 // calculate a * b = r0r1
using namespace std;
// calculate a * b = r0r1
  template<typename T>
  typename std::enable_if<std::is_unsigned<T>::value,tuple<T,T>>::type
  constexpr long_mul(const T& a, const T& b){
    const T N  = (a)/2;
    const T t0 = (a>>N)*(b>>N);
    const T t1 = ((a<<N)>>N)*(b>>N);
    const T t2 = (a>>N)*((b<<N)>>N);
    const T t3 = ((a<<N)>>N)*((b<<N)>>N);
    const T t4 = t3+(t1<<N);
    const T r1 = t4+(t2<<N);
    const T r0 = (r1<t4)+(t4<t3)+(t1>>N)+(t2>>N)+t0;
    return {r0,r1};
  }




// C++11 constexpr functions use recursion rather than iteration
// (C++14 constexpr functions may use local variables and loops)
constexpr int factorial(int n)
{
    return n <= 1 ? 1 : (n * factorial(n - 1));
}
 
// literal class
class conststr {
    const char* p;
    std::size_t sz;
public:
    template<std::size_t N>
    constexpr conststr(const char(&a)[N]): p(a), sz(N - 1) {}
 
    // constexpr functions signal errors by throwing exceptions
    // in C++11, they must do so from the conditional operator ?:
    constexpr char operator[](std::size_t n) const
    {
        return n < sz ? p[n] : throw std::out_of_range("");
    }
    constexpr std::size_t size() const { return sz; }
};
 
// C++11 constexpr functions had to put everything in a single return statement
// (C++14 doesn't have that requirement)
constexpr std::size_t countlower(conststr s, std::size_t n = 0,
                                             std::size_t c = 0)
{
    return n == s.size() ? c :
           'a' <= s[n] && s[n] <= 'z' ? countlower(s, n + 1, c + 1) :
                                       countlower(s, n + 1, c);
}
 
// output function that requires a compile-time constant, for testing
template<int n>
struct constN
{
    constN() { std::cout << n << '\n'; }
};
 
int main()
{
   uint8_t lut[256] = { 255u,254u,253u,252u,251u,250u,249u,248u,247u,246u,245u,244u,243u,242u,241u,
240u,240u,239u,238u,237u,236u,235u,234u,234u,233u,232u,231u,230u,229u,229u,228u,
227u,226u,225u,225u,224u,223u,222u,222u,221u,220u,219u,219u,218u,217u,217u,216u,
215u,214u,214u,213u,212u,212u,211u,210u,210u,209u,208u,208u,207u,206u,206u,205u,
204u,204u,203u,202u,202u,201u,201u,200u,199u,199u,198u,197u,197u,196u,196u,195u,
195u,194u,193u,193u,192u,192u,191u,191u,190u,189u,189u,188u,188u,187u,187u,186u,
186u,185u,185u,184u,184u,183u,183u,182u,182u,181u,181u,180u,180u,179u,179u,178u,
178u,177u,177u,176u,176u,175u,175u,174u,174u,173u,173u,172u,172u,172u,171u,171u,
170u,170u,169u,169u,168u,168u,168u,167u,167u,166u,166u,165u,165u,165u,164u,164u,
163u,163u,163u,162u,162u,161u,161u,161u,160u,160u,159u,159u,159u,158u,158u,157u,
157u,157u,156u,156u,156u,155u,155u,154u,154u,154u,153u,153u,153u,152u,152u,152u,
151u,151u,151u,150u,150u,149u,149u,149u,148u,148u,148u,147u,147u,147u,146u,146u,
146u,145u,145u,145u,144u,144u,144u,144u,143u,143u,143u,142u,142u,142u,141u,141u,
141u,140u,140u,140u,140u,139u,139u,139u,138u,138u,138u,137u,137u,137u,137u,136u,
136u,136u,135u,135u,135u,135u,134u,134u,134u,134u,133u,133u,133u,132u,132u,132u,
132u,131u,131u,131u,131u,130u,130u,130u,130u,129u,129u,129u,129u,128u,128u,128u,
127u
    };
   
    std::cout << "4! = " ;
    constN<factorial(4)> out1; // computed at compile time
 
    volatile int k = 8; // disallow optimization using volatile
    std::cout << k << "! = " << factorial(k) << '\n'; // computed at run time
 
    std::cout << "the number of lowercase letters in \"Hello, world!\" is ";
    constN<countlower("Hello, world!")> out2; // implicitly converted to conststr
    uint32_t a=32, b=32;
    uint32_t d=3;

    const auto l = int(log2(d));

    uint32_t x;
    if (l<8) {
      x = 1<<((d)-1-l);
    } else {
      if ((d)>(l+8)) x = (lut[(d>>(l-8))-256])<<((d)-l-8);
      else x = (lut[(d>>(l-8))-256])>>(l+8-(d));
    }
 uint32_t i =0;
if (x==0) x=1;
    while(true) {
      const auto lm = long_mul(x+1,i-x*d);
        i = get<0>(lm);
      if (i) x+=i;
      else break ;
    }

    const auto lm = long_mul<uint32_t>(a,b);
    const uint32_t i = get<0>(lm);
    cout<<"T="<<i<<", "<<  get<1>(lm)<<"x="<<x<<endl;

}
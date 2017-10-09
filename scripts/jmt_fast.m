function [coeffs, sT] = jmt_fast(sk,sk_dot,sk_double_dot,...
 sT_dot, sT_double_dot,T)

if abs(T)<=0.3
warning('Time horizon is too short. May lead to inexact calculation')
end

s_lim =[          - ( sk + sk_dot * T + sk_double_dot/2 * T*T ) ;
      sT_dot      - ( 0  + sk_dot     + sk_double_dot   * T    );
    sT_double_dot - ( 0  + 0          + sk_double_dot          ) ];

Tpk =[-1, T*T*T    , T*T*T*T     ;
      0,  3 * T*T  , 4 * T*T*T   ;
      0,  6 * T    , 12 * T*T    ];

ak = Tpk\ (s_lim);
sT = ak(1);
coeffs = [sk, sk_dot, sk_double_dot/2, ak(2), ak(3)];

coeffs = coeffs(end:-1:1);

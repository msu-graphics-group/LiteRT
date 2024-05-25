#include "equation_solvers.h"

void
solver::find_interval(const float* coefs, const float& x1, const float& x2, const int& n, float interval[2])
{
    int nb = n, nroot = 0;
    float dx = (x2 - x1) / (n * 1000);
    float x = x1;
    float fp = f(coefs, x1, n);

    bool go_further = true;

    for (int i = 0; i < n * 1000 && go_further; i++)
    {
        x += dx;
        float fc = f(coefs, x, n);

        if (fc * fp <= 0.f)
        {
            // std::cout << x - dx << " " << x << std::endl;

            interval[0] = x - dx;
            interval[1] = x;

            go_further = false;
        }

        fp = fc;
    }
}

float 
solver::f(const float* coefs, const float& x, const int& n)
{
    float res = 0;

    for (int i = 0; i <= n; i++)
    {
        res += std::pow(x, i) * coefs[i];
    }

    return res;
}

float
solver::df(const float* coefs, const float& x, const int& n)
{
    float res = 0;

    for (int i = 1; i <= n; i++)
    {
        res += coefs[i] * i * std::pow(x, i - 1);
    }

    return res;
}

void 
solver::polinomMiltiplier(const float* p1, const float* p2, float* res, const int& n1, const int& n2, const float& polinom_coef)
{
    for (int i = 0; i < n1; i++)
    {
        for (int j = 0; j < n2; j++)
        {
            res[i + j] += polinom_coef * p1[i] * p2[j];
        }
    }
}

void
solver::coefsDecrease(const float* coefs, const LiteMath::float3& P, const LiteMath::float3& D, float* res)
{
    float x_polinoms[4][4] = {{1, 0, 0, 0}, {P.x, D.x, 0, 0}, {P.x * P.x, 2 * P.x * D.x, D.x * D.x, 0}, {P.x * P.x * P.x, 3 * P.x * P.x * D.x, 3 * P.x * D.x * D.x, D.x * D.x * D.x}};
    float y_polinoms[4][4] = {{1, 0, 0, 0}, {P.y, D.y, 0, 0}, {P.y * P.y, 2 * P.y * D.y, D.y * D.y, 0}, {P.y * P.y * P.y, 3 * P.y * P.y * D.y, 3 * P.y * D.y * D.y, D.y * D.y * D.y}};
    float z_polinoms[4][4] = {{1, 0, 0, 0}, {P.z, D.z, 0, 0}, {P.z * P.z, 2 * P.z * D.z, D.z * D.z, 0}, {P.z * P.z * P.z, 3 * P.z * P.z * D.z, 3 * P.z * D.z * D.z, D.z * D.z * D.z}};

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            float tmp[10] = {0};
            solver::polinomMiltiplier(x_polinoms[i], y_polinoms[j], tmp, i + 1, j + 1, 1);
            
            for (int k = 0; k < 4; k++)
            {
                solver::polinomMiltiplier(tmp, z_polinoms[k], res, i + j + 1, k + 1, coefs[i + 4 * j + 16 * k]);
            }
        }
    }
}

float 
solver::calc_test_res(const std::vector<float>& coefs, const LiteMath::float3& P, const LiteMath::float3& D, const float& t)
{
    float power_x[4], power_y[4], power_z[4];
    float res = 0;
    float tx = P.x + t * D.x, ty = P.y + t * D.y, tz = P.z + t * D.z;

    power_x[0] = 1;
    power_x[1] = tx;
    power_x[2] = tx * tx;
    power_x[3] = tx * tx * tx;

    power_y[0] = 1;
    power_y[1] = ty;
    power_y[2] = ty * ty;
    power_y[3] = ty * ty * ty;

    power_z[0] = 1;
    power_z[1] = tz;
    power_z[2] = tz * tz;
    power_z[3] = tz * tz * tz;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                res += coefs[i + 4 * j + 16 * k] * power_x[i] * power_y[j] * power_z[k];
            }
        }
    }

    return res;
}

float 
solver::nr_solver(const float* coefs, const float& x1, const float& x2, const float& acc, bool& has_intersection)
{
    const int MAXIT = 100;
    float xh = 0, xl = 0;
    float fl = solver::f(coefs, x1, 9), fh = solver::f(coefs, x2, 9);

    has_intersection = false;
    // if ((fl > 0.0 && fh > 0.0) || (fl < 0.0 && fh < 0.0))
    // {
    //     throw("Root must be bracketed");
    // }

    if (fl == 0.0) return x1;
    if (fh == 0.0) return x2;

    if (fl < 0.0)
    {
    
        xl = x1;
        xh = x2;
    }
    else
    {
        xh = x1;
        xl = x2;
    }

    float rts = 0.5 * (x2 + x1), dxold = std::abs(x2 - x1), dx = dxold, f = solver::f(coefs, rts, 9), df = solver::df(coefs, rts, 9);

    for (int i = 0; i < MAXIT; i++)
    {
        if ((((rts-xh)*df-f)*((rts-xl)*df-f) > 0.0) || (std::abs(2.0*f) > std::abs(dxold*df)))
        {
            dxold = dx;
            dx = 0.5 * (xh - xl);
            rts = xl + dx;

            if (xl == rts)
            {
                has_intersection = true;
                return rts;
            }
        }
        else
        {
            dxold=dx;
            dx=f/df;
            float temp=rts;
            rts -= dx;
            if (temp == rts)
            {
                has_intersection = true;
                return rts;
            }
        }

        if (std::abs(dx) < acc)
        {
            has_intersection = true;
            return rts;
        }

        f=solver::f(coefs, rts, 9);
        df=solver::df(coefs, rts, 9);

        if (f < 0.0)
        {
            xl = rts;
        }
        else
        {
            xh = rts;
        }
    }

    // throw("Maximum number of iterations exceeded");
    return 0;
}

float 
solver::ddf(const float* coefs, const float& x, const int& n)
{
    float res = 0;

    for (int i = 2; i <= n; i++)
    {
        res += coefs[i] * i * (i - 1) * std::pow(x, i - 2);
    }

    return res;
}

float 
solver::halley_solver(const float* coefs, const float& x1, const float& x2, const float& acc)
{
    const int MAXIT = 100;
    float xh = 0, xl = 0;
    float fl = solver::f(coefs, x1, 9), fh = solver::f(coefs, x2, 9);

    if ((fl > 0.0 && fh > 0.0) || (fl < 0.0 && fh < 0.0))
    {
        throw("Root must be bracketed");
    }

    if (fl == 0.0) return x1;
    if (fh == 0.0) return x2;

    if (fl < 0.0)
    {
    
        xl = x1;
        xh = x2;
    }
    else
    {
        xh = x1;
        xl = x2;
    }

    float rts = 0.5 * (x2 + x1), dxold = std::abs(x2 - x1), dx = dxold, f = solver::f(coefs, rts, 9), df = solver::df(coefs, rts, 9), ddf = solver::ddf(coefs, rts, 9);

    for (int i = 0; i < MAXIT; i++)
    {
        if ((((rts-xh)*df-f)*((rts-xl)*df-f) > 0.0) || (std::abs(2.0*f) > std::abs(dxold*df)))
        {
            dxold = dx;
            dx = 0.5 * (xh - xl);
            rts = xl + dx;

            if (xl == rts) return rts;
        }
        else
        {
            dxold=dx;
            dx=f/(df * std::max(0.8, std::min(1.2, 1 - 0.5 * f * ddf / (df * df))));
            float temp=rts;
            rts -= dx;
            if (temp == rts) return rts;
        }

        if (std::abs(dx) < acc) return rts;

        f = solver::f(coefs, rts, 9);
        df = solver::df(coefs, rts, 9);
        ddf = solver::ddf(coefs, rts, 9);

        if (f < 0.0)
        {
            xl = rts;
        }
        else
        {
            xh = rts;
        }
    }

    throw("Maximum number of iterations exceeded");
}

float 
solver::bisection_solver(const float* coefs, const float& x1, const float& x2, const float& acc)
{
    const int JMAX = 50;
    float dx = 0, xmid = 0, rtb = 0;
    float f = solver::f(coefs, x1, 9), fmid = solver::f(coefs, x2, 9);

    if (f*fmid >= 0.0) throw("Root must be bracketed for bisection");

    rtb = f < 0.0 ? (dx=x2-x1,x1) : (dx=x1-x2,x2);

    for (int j=0;j<JMAX;j++) 
    {
        fmid=solver::f(coefs, xmid=rtb+(dx *= 0.5), 9);
        if (fmid <= 0.0) rtb=xmid;
        if (std::abs(dx) < acc || fmid == 0.0) return rtb;
    }

    throw("Too many bisections");
}

float 
solver::brent_solver(const float* coefs, const float& x1, const float& x2, const float& acc)
{
    const int ITMAX=100;
    const float EPS=std::numeric_limits<float>::epsilon();
    float a=x1,b=x2,c=x2,d,e,fa=solver::f(coefs, a, 9),fb=solver::f(coefs, b, 9),fc,p,q,r,s,tol1,xm;

    if ((fa > 0.0 && fb > 0.0) || (fa < 0.0 && fb < 0.0))
        throw("Root must be bracketed");

    fc=fb;

    for (int iter=0;iter<ITMAX;iter++)
    {
        if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0))
        {
            c=a;
            fc=fa;
            e=b-a;
            d=b-a;
        }

        if (std::abs(fc) < std::abs(fb)) 
        {
            a=b;
            b=c;
            c=a;
            fa=fb;
            fb=fc;
            fc=fa;
        }

        tol1=2.0*EPS*std::abs(b)+0.5*acc;
        xm=0.5*(c-b);
        if (std::abs(xm) <= tol1 || fb == 0.0) return b;
        if (std::abs(e) >= tol1 && std::abs(fa) > std::abs(fb)) 
        {
            s=fb/fa;
            if (a == c) 
            {
                p=2.0*xm*s;
                q=1.0-s;
            } 
            else 
            {
                q=fa/fc;
                r=fb/fc;
                p=s*(2.0*xm*q*(q-r)-(b-a)*(r-1.0));
                q=(q-1.0)*(r-1.0)*(s-1.0);
            }

            if (p > 0.0) q = -q; 
            p=std::abs(p);

            float min1=3.0*xm*q-std::abs(tol1*q);
            float min2=std::abs(e*q);

            if (2.0*p < (min1 < min2 ? min1 : min2)) 
            {
                e=d;
                d=p/q;
            }
            else
            {
                d=xm;
                e=d;
            }
        }
        else
        {
            d=xm;
            e=d;
        }

        a=b;
        fa=fb;

        if (std::abs(d) > tol1)
        {
            b += d;
        }
        else
        {
            b += (xm > 0) ? std::abs(tol1) : -std::abs(tol1);
            fb=solver::f(coefs, b, 9);
        }
    }

    throw("Maximum number of iterations exceeded");
}
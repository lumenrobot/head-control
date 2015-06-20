using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Numerics;
using System.Collections;
namespace LumenMotion
{
    
    public class mf
    {
        public double[] value;
        public double[] x;
        public string name;
        public int lenght;
        public double c, sigma;
        public mf(string name, double[] x, double c, double sigma)
        {
            this.name = name;
            this.lenght = x.Length;
            this.x = x;
            this.c = c;
            this.sigma = sigma;
            value = new double[x.Length];
            for (int i = 0; i < value.Length; i++)
            {
                value[i] = Math.Exp((-0.5 * ((x[i] - c) / sigma) * ((x[i] - c) / sigma)));
            }
        }
        public mf(int lenght)
        {
            this.lenght = lenght;
            value = new double[this.lenght];
            for (int i = 0; i < this.lenght; i++)
            {
                value[i] = 0;
            }
        }
        public double getValue(double element)
        {

            return Math.Exp((-0.5 * ((element - c) / sigma) * ((element - c) / sigma)));
        }
    }
    public class FuzzyOperation
    {
        public mf max(mf a, mf b)
        {
            mf output = new mf(a.lenght);
            for (int i = 0; i < output.lenght; i++)
            {
                output.value[i] = Math.Max(a.value[i], b.value[i]);
            }
            return output;
        }
        public mf max(mf a, double x)
        {
            mf output = new mf(a.lenght);
            for (int i = 0; i < output.lenght; i++)
            {
                output.value[i] = Math.Max(a.value[i], x);
            }
            return output;
        }
        public mf max(mf a, double x, double y)
        {
            mf output = new mf(a.lenght);
            double min = Math.Min(x, y);
            for (int i = 0; i < output.lenght; i++)
            {
                output.value[i] = Math.Max(a.value[i], min);
            }
            return output;
        }
        public mf min(mf a, mf b)
        {
            mf output = new mf(a.lenght);
            for (int i = 0; i < output.lenght; i++)
            {
                output.value[i] = Math.Min(a.value[i], b.value[i]);
            }
            return output;
        }
        public mf min(mf a, double x)
        {
            mf output = new mf(a.lenght);
            for (int i = 0; i < output.lenght; i++)
            {
                output.value[i] = Math.Min(a.value[i],x);
            }
            return output;
        }
        public mf min(mf a, double x,double y)
        {
            mf output = new mf(a.lenght);
            double min = Math.Min(x, y);
            for (int i = 0; i < output.lenght; i++)
            {
                output.value[i] = Math.Min(a.value[i], min);
            }
            return output;
        }
        public double centroid(mf a)
        {
            double output;
            double x = 0;
            double y = 0;
            for (int i = 0; i < a.lenght; i++)
            {
                y = y + a.value[i];
                x = x + a.value[i] * a.x[i];
            }
            output = x / y;
            return output;
        }
        public mf agregator(List<mf> input)
        {
            mf output = new mf(input[0].lenght);

            for (int i = 0; i < input.Count; i++)
            {
                output = this.max(input[i], output);
                
            }
            
            return output;
        }
    }
}

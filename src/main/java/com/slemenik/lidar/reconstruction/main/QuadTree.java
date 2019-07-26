
package com.slemenik.lidar.reconstruction.main;

import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;


/** This implements a versatile QuadTree structure.  Not thread-safe.
 *
 * Contains the following dynamic parameters:
 *
 * - LEAF_MAX_OBJECTS  (default 10) is the maximum number of items stored in one leaf (splitting occurs with overflow);
 * - DYNAMIC_MAX_OBJECTS  (default false) tells whether adjusting LEAF_MAX_OBJECTS automatically is enabled;
 * - MAX_OBJ_TARGET_EXPONENT  (default 0.33333) is used in dynamically adjusting LEAF_MAX_OBJECTS, if that is enabled –
 *  LEAF_MAX_OBJECTS is calculated by the formula SIZE ^ MAX_OBJ_TARGET_EXPONENT,
 *  and a lower minimum of 7 is applied on top of that.
 *
 * @author pvto https://github.com/pvto
 */
public class QuadTree<T> {

    public int LEAF_MAX_OBJECTS = 10;
    public boolean DYNAMIC_MAX_OBJECTS = false;
    public double MAX_OBJ_TARGET_EXPONENT = 0.333333;
    private int size = 0;
    public ListProvider<CoordHolder> LIST_PROVIDER = ListProvider.LP_LINKEDLIST;

    public class CoordHolder {
        public double x,y;
        public T o;
        public Quad quad;
        public CoordHolder(double x, double y, T o, Quad quad) {
            this.x = x;
            this.y = y;
            this.o = o;
            this.quad = quad;
        }
        public void replace()
        {
            quad.replace(this, 0);
        }
        public void remove()
        {
            if (quad.items.remove(this))
                size--;
        }
        public int depth()
        {
            int i = 0;
            Quad q = this.quad;
            while(q != null)
            {
                i++;
                q = q.parent;
            }
            return i;
        }
    }


    public class Quad {
        public Quad
                parent = null,
                UL = null, // upper left corner child ...
                UR = null,
                LL = null,
                LR = null
                ;
        public double
                x1,y1,
                x2,y2
                ;
        public List<CoordHolder> items = LIST_PROVIDER.getList(LEAF_MAX_OBJECTS, size);


        public Quad(Quad parent, double x1, double y1, double x2, double y2)
        {
            this.parent = parent;
            this.x1 = x1;  this.y1 = y1;
            this.x2 = x2;  this.y2 = y2;
        }

        public void findAll(double X1, double Y1, double X2, double Y2, List<CoordHolder> ret)
        {
            if (UL == null)
            {
                for(CoordHolder h : items)
                    if (h.x >= X1 && h.y >= Y1 && h.x <= X2 && h.y <= Y2)
                        ret.add(h);
                return;
            }
            if (overlap(UL, X1, Y1, X2, Y2)) UL.findAll(X1, Y1, X2, Y2, ret);
            if (overlap(UR, X1, Y1, X2, Y2)) UR.findAll(X1, Y1, X2, Y2, ret);
            if (overlap(LL, X1, Y1, X2, Y2)) LL.findAll(X1, Y1, X2, Y2, ret);
            if (overlap(LR, X1, Y1, X2, Y2)) LR.findAll(X1, Y1, X2, Y2, ret);
        }

        private boolean overlap(Quad q, double X1, double Y1, double X2, double Y2)
        {
            if (q.x2 < X1 || q.y2 < Y1 || q.x1 > X2 || q.y1 > Y2) return false;
            return true;
        }

        public CoordHolder place(double x, double y, T o)
        {
            return place(new CoordHolder(x, y, o, null), 0);
        }

        public CoordHolder place(CoordHolder h, int n)
        {
            double x = h.x,
                    y = h.y
                    ;
            if (x < x1 || y < y1 || x > x2 || y > y2) {
                if (x1 == x2)
                {
                    x1 = Math.min(x1, x);
                    y1 = Math.min(y1, y);
                    double add = Math.max(Math.max(x2, x) - x1, Math.max(y2, y) - y1);
                    x2 = x1 + add;
                    y2 = y1 + add;
                }
                else
                {
                    if (parent == null)
                    {
                        initParent(x, y);
                    }
                    return parent.place(h, n+1);
                }
            }
            if (items.size() == LEAF_MAX_OBJECTS)
            {
                expand(n+1);
            }
            if (UL != null)
            {
                return place_(h, this, n+1);
            }
            else
            {
                h.quad = this;
                items.add(h);
                return h;
            }
        }

        private CoordHolder place_(CoordHolder h, Quad quad, int n)
        {
            while (quad.UL != null)
            {
                if (h.x <= (quad.x2 + quad.x1) / 2)
                {
                    quad = (h.y <= (quad.y2 + quad.y1) / 2 ? quad.UL : quad.LL);
                }
                else
                {
                    quad = (h.y <= (quad.y2 + quad.y1) / 2 ? quad.UR : quad.LR);
                }
            }
            return quad.place(h, n+1);
        }

        private void expand(int n)
        {
            if (LL == null)
            {
                initQuad();
            }
            for(CoordHolder c : items)
            {
                place_(c, this, n+1);
            }
            items = Collections.EMPTY_LIST;
        }

        private void initQuad()
        {
            UL = new Quad(this, x1, y1, (x2 + x1) / 2.0, (y2 + y1) / 2.0);
            UR = new Quad(this, (x2 + x1) / 2.0, y1, x2, (y2 + y1) / 2.0);
            LL = new Quad(this, x1, (y2 + y1) / 2.0, (x2 + x1) / 2.0, y2);
            LR = new Quad(this, (x2 + x1) / 2.0, (y2 + y1) / 2.0, x2, y2);
        }

        private Quad initParent(double x, double y)
        {
            int quadInd = 0;
            double
                    X1 = x1,
                    Y1 = y1,
                    X2 = x2 + (x2 - x1),
                    Y2 = y2 + (y2 - y1)
                    ;
            if (x < X1)
            {
                quadInd++;
                X1 -= (x2 - x1);
                X2 -= (x2 - x1);
            }
            if (y < Y1)
            {
                quadInd += 2;
                Y1 -= (y2 - y1);
                Y2 -= (y2 - y1);
            }
            parent = new Quad(null, X1, Y1, X2, Y2);
            parent.initQuad();
            switch(quadInd) {
                case 0: parent.UL = this; break;
                case 1: parent.UR = this; break;
                case 2: parent.LL = this; break;
                case 3: parent.LR = this; break;
            }
            root = parent;
            return parent;
        }

        public void replace(CoordHolder item, int n)
        {
            if (item.x < x1 || item.y < y1 || item.x > x2 || item.y > y2)
            {
                items.remove(item);
                if (parent == null)
                {
                    initParent(item.x, item.y);
                }
                parent.place(item, n+1);
            }
        }

        private void printChar(PrintStream out, char c, int n)
        {
            for(int i = 0; i < n; i++) out.print(c);
        }

        private void print(PrintStream out, int indent)
        {
            printChar(out, '.', indent - 1); out.print(' ');
            out.print('(');
            out.print(x1);  out.print(',');  out.print(y1);
            out.print(" - ");
            out.print(x2);  out.print(',');  out.print(y2);
            out.println(')');

            for(CoordHolder h : this.items)
            {
                printChar(out, ' ', indent + 2);
                out.print(h.x);
                out.print(',');
                out.print(h.y);
                out.print(": ");
                out.println(h.o.toString().replaceAll("\r?\n.*", ""));
            }
            if (UL != null)
            {
                printChar(out, ' ', indent);
                out.println('[');
                UL.print(out, indent + 2);
                UR.print(out, indent + 2);
                LL.print(out, indent + 2);
                LR.print(out, indent + 2);
                printChar(out, ' ', indent);
                out.println(']');
            }
        }

    }

    public Quad root;

    public List<CoordHolder> findAll(double x1, double y1, double x2, double y2)
    {
        List<CoordHolder> ret = new ArrayList<CoordHolder>();
        root.findAll(x1, y1, x2, y2, ret);
        return ret;
    }

    public CoordHolder place(double x, double y, T o)
    {
        if (root == null)
        {
            root = new Quad(null, x, y, x, y);
        }
        CoordHolder h = root.place(x, y, o);
        size++;
        if (DYNAMIC_MAX_OBJECTS && size % 100 == 0)
        {
            adjustMaxObjects();
        }
        return h;
    }

    public void adjustMaxObjects()
    {
        this.LEAF_MAX_OBJECTS = Math.max(7,
                (int)Math.pow(size, MAX_OBJ_TARGET_EXPONENT));
    }

    public int size() { return size; }

    public void print(PrintStream out)
    {
        root.print(out, 0);
    }

}

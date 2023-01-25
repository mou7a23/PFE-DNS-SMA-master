public class Vecteur2D {
    // coordinates
    private double x;
    private double y;

    public Vecteur2D(){
        x = .0;
        y = .0;
    }

    public Vecteur2D(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Vecteur2D(double theta){
        this.x = Math.cos(theta);
        this.y = Math.sin(theta);
    }

    double get_x(){
        return x;
    }

    double get_y(){
        return y;
    }

    double get_norm(){
        return Math.sqrt(x * x + y * y);
    }

    double get_angle(){
        return Math.atan2(y, x);
    }
}

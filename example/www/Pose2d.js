class Pose2d {
    constructor(data = [0, 0, 0, 0, 0, 0]) {
        this._data = data;
    }

    get position() {
        return this._data.slice(0, 2);
    }

    set position(value) {
        this._data[0] = value[0];
        this._data[1] = value[1];
    }

    get velocity() {
        return this._data.slice(2, 4);
    }

    set velocity(value) {
        this._data[2] = value[0];
        this._data[3] = value[1];
    }

    get acceleration() {
        return this._data.slice(4, 6);
    }

    set acceleration(value) {
        this._data[4] = value[0];
        this._data[5] = value[1];
    }

    get heading() {
        return Math.atan2(this._data[3], this._data[2]);
    }

    get curvature() {
        return (this._data[2] * this._data[5] - this._data[4] * this._data[3]) / (this.speed * this.speed * this.speed);
    }

    get speed() {
        return Math.sqrt(this._data[2] * this._data[2] + this._data[3] * this._data[3]);
    }

    get x() {
        return this._data[0];
    }

    get y() {
        return this._data[1];
    }

    get dx() {
        return this._data[2];
    }

    get dy() {
        return this._data[3];
    }

    get ddx() {
        return this._data[4];
    }

    get ddy() {
        return this._data[5];
    }

    set x(val) {
        this._data[0] = val;
    }

    set y(val) {
        this._data[1] = val;
    }

    set dx(val) {
        this._data[2] = val;
    }

    set dy(val) {
        this._data[3] = val;
    }

    set ddx(val) {
        this._data[4] = val;
    }

    set ddy(val) {
        this._data[5] = val;
    }

    clone() {
        return new Pose2d(this._data.slice());
    }

    to_wp() {
        return {
            point: [this.x, this.y],
            tangent: [this.dx, this.dy],
            curvature: [this.ddx, this.ddy]
        }
    }
}

export default Pose2d;
import Pose2d from './Pose2d.js';

var field_width = 323.25;
var field_length = 629.25;
var field_box = [144, 20, 854, 384];
var heading_weight = 35;

var px_per_in = [(field_box[2] - field_box[0]) / field_length, (field_box[3] - field_box[1]) / field_width, 0];
px_per_in[2] = (px_per_in[0] + px_per_in[1]) / 2;

var axes = {
    x: 0,
    y: 1
};

function clamp(x, axis, box = field_box) {
    return Math.max(Math.min(x, box[axis + 2]), box[axis]);
}

function px_to_in(x, axis) {
    if (axis != undefined) {
        return Math.round((clamp(x, axis) - field_box[axis]) / px_per_in[axis]);
    }

    return Math.round(x / px_per_in[2]);
}

class PoseTable {
    constructor(container) {
        this.container = $(container);
        this.tablePoses = [
            new Pose2d([509, 230, -35, 0, 0, 0])
        ];

        this.tablePoses[0].poseIndex = 0;
    }

    convert(pose) {
        var newPose = new Pose2d();
        newPose.x = px_to_in(pose.x, axes.x);
        newPose.y = px_to_in(pose.y, axes.y);
        newPose.dx = px_to_in(pose.dx);
        newPose.dy = px_to_in(pose.dy);
        newPose.ddx = px_to_in(pose.ddx);
        newPose.ddy = px_to_in(pose.ddy);
        newPose.poseIndex = pose.poseIndex;

        return newPose;
    }

    setCanvas(canvas) {
        this.canvas = canvas;

        this.canvas.setPoses(this.tablePoses);
    }

    reset() {
        var removables = this.tablePoses.slice(1, this.tablePoses.length);
        removables.map((v, i) => this.deleteRow(1));

        this.tablePoses = this.tablePoses.slice(0, 1);

        this.canvas.setPoses(this.tablePoses);
    }

    resetCurvature() {
        this.tablePoses.forEach(e => {
            e.ddx = 0;
            e.ddy = 0;
        });

        this.canvas.setPoses(this.tablePoses);
    }

    refresh() {
        var poses = [];
        this.container.children().each((i, v) => {
            var row = $(v).children('input').map((j, e) => parseInt($(e).val()));
            var rad = Math.PI / 180 * row[2];
            row[3] = Math.max(row[3], 1);
            var pose = new Pose2d([row[0], row[1], heading_weight * Math.cos(rad), heading_weight * Math.sin(rad), 0, 0]);
            pose.poseIndex = i;
            poses.push(pose);
        });

        this.tablePoses = poses;
        this.canvas.setPoses(this.tablePoses);
    }

    setOrigin(coords) {
        this.tablePoses[0].position = coords;
        this.tablePoses[0].velocity = [-35, 0];

        this.update(this.tablePoses[0]);

        this.canvas.setPoses(this.tablePoses);
    }

    setOffsets(wps) {
        for (var i = 1; i < this.tablePoses.length - 1; i++) {
            this.tablePoses[i].ddx = wps[(i - 1) * 2];
            this.tablePoses[i].ddy = wps[(i - 1) * 2 + 1];
        }

        this.canvas.setPoses(this.tablePoses);
    }

    addPose(pose) {
        this.addRow();
        this.tablePoses.push(this.convert(pose));

        this.update(this.tablePoses[pose.poseIndex]);
    }

    updateFromCanvas(canvasPose) {
        var pose = this.convert(canvasPose);
        this.tablePoses[pose.poseIndex] = pose;

        this.update(pose);
    }

    update(pose) {
        var child = this.container.children().filter(i => i == pose.poseIndex);
        child.children('#x').val(pose.x);
        child.children('#y').val(pose.y);
        child.children('#head').val(Math.round(180 / Math.PI * pose.heading));
    }

    remove(row) {
        var index = $(row).parent().index();
        this.deleteRow(index);

        this.canvas.setPoses(this.tablePoses);
    }

    addRow() {
        var clone = this.container.children().first().clone(true);

        $(this.container).append(clone);
    }

    deleteRow(row) {
        var kids = this.container.children();
        if (kids.length > 1) {
            this.container.children()[row].remove();

            this.tablePoses = this.tablePoses.filter((v, i) => i != row).map((v, i) => {
                v.poseIndex = i;
                return v;
            });
        }
    }

    waypoints() {
        var poses = [];
        for (var i = 0; i < this.tablePoses.length; i++) {
            var tmp = this.tablePoses[i].clone();
            tmp.x -= this.tablePoses[0].x;
            tmp.y -= this.tablePoses[0].y;

            poses.push(tmp);
        }

        return poses;
    }
}

export default PoseTable;
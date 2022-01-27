import Pose2d from './Pose2d.js';
import init, { wps_to_cheesy_path, wps_to_jaci_path, optimize } from "./wayfinder_wasm.js";

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
var fine_jaci_params = { max_ds: 0.1, max_dc: 0.01 };
var sparse_jaci_params = { max_ds: 0.1, max_dc: 0.01 };
var fine_cheesy_params = { max_dx: 1, max_dy: 1, max_dt: 1 };
var sparse_cheesy_params = { max_dx: 2, max_dy: 1, max_dt: 1 }


var sparse_params = sparse_cheesy_params;
var fine_params = fine_cheesy_params;
var opt_param = {
    eps: 1e-4,
    samples: 10,
    max_iters: 10,
    target_cost: 0.0
};


function clamp(x, axis, box = field_box) {
    return Math.max(Math.min(x, box[axis + 2]), box[axis]);
}

function in_to_px(x, axis) {
    if (axis != undefined) {
        return clamp(x * px_per_in[axis] + field_box[axis], axis);
    }

    return x * px_per_in[2];
}

function px_to_in(x, axis) {
    if (axis != undefined) {
        return Math.round((clamp(x, axis) - field_box[axis]) / px_per_in[axis]);
    }

    return Math.round(x / px_per_in[2]);
}

function into_array(arr) {
    var pts = [];
    for (var i = 0; i < arr.length; i++) {
        pts.push([arr[i].position[0], arr[i].position[1]]);
    }

    return pts;
}


function to_wps(pts) {
    var wps = [];
    for (var i = 0; i < pts.length; i++) {
        wps.push(pts[i].to_wp());
    }

    for (var i = 0; i < wps.length; i++) {
        var dist;
        if (i == 0) {
            dist = Math.sqrt(Math.pow((wps[i + 1].point[0] - wps[i].point[0]), 2) + Math.pow((wps[i + 1].point[1] - wps[i].point[1]), 2));
        } else if (i == wps.length - 1) {
            dist = Math.sqrt(Math.pow((wps[i].point[0] - wps[i - 1].point[0]), 2) + Math.pow((wps[i].point[1] - wps[i - 1].point[1]), 2));
        } else {
            dist = (Math.sqrt(Math.pow((wps[i + 1].point[0] - wps[i].point[0]), 2) + Math.pow((wps[i + 1].point[1] - wps[i].point[1]), 2)) + Math.sqrt(Math.pow((wps[i].point[0] - wps[i - 1].point[0]), 2) + Math.pow((wps[i].point[1] - wps[i - 1].point[1]), 2))) / 2;
        }
        var v_mag = Math.sqrt(Math.pow(wps[i].tangent[0], 2) + Math.pow(wps[i].tangent[1], 2));
        wps[i].tangent = [wps[i].tangent[0] / v_mag * dist, wps[i].tangent[1] / v_mag * dist];
    }
    return wps;
}

class Field {
    constructor(container) {
        this.width = 1000;
        this.height = 406;
        this.activeClass = 'active-d3-item';
        this.robotLength = 34;
        this.robotWidth = 30;
        this.motion = [];
        this.poses = [];

        init()

        this.posDragBehavior = d3.drag()
            .on('start', d => this.startDragPos(d))
            .on('drag', d => this.draggedPos(d))
            .on('end', d => this.endDragPos(d));

        this.velDragBehavior = d3.drag()
            .on('start', d => this.startDragVel(d))
            .on('drag', d => this.draggedVel(d))
            .on('end', d => this.endDragVel(d));

        this.imgDragBehavior = d3.drag()
            .on('start', d => this.startDragImg(d))
            .on('drag', d => this.draggedImg(d))
            .on('end', d => this.endDragImg(d));

        this.canvas = d3.select(container)
            .append('svg')
            .attr('width', this.width)
            .attr('height', this.height);

        this.canvas.append('svg:image')
            .attr('x', 0)
            .attr('y', 0)
            .attr('width', this.width)
            .attr('height', this.height)
            .attr('xlink:href', 'assets/2020-field.png')
            .call(this.imgDragBehavior);

        this.lines = this.canvas
            .selectAll('line');

        this.posPoints = this.canvas
            .selectAll('rect');

        this.velPoints = this.canvas
            .selectAll('circle');

        this.path = this.canvas
            .append('path');
    }

    setTable(table) {
        this.table = table;
    }

    updatePath() {
        var wps = to_wps(this.poses);
        var out = wps_to_cheesy_path(wps, sparse_params);
        this.motion = into_array(out);

        this.path
            .datum(this.motion)
            .attr("fill", "none")
            .attr("stroke", "orchid")
            .attr("stroke-width", 1.5)
            .attr("stroke-linejoin", "round")
            .attr("stroke-linecap", "round")
            .attr("d", d3.line());
    }

    optimizePath() {
        var wps = to_wps(this.poses);
        var opt_wps = optimize(wps, opt_param);
        this.setOffsets(opt_wps);
    }

    setOffsets(data) {
        for (var i = 0; i < data.length; i += 2) {
            this.poses[i / 2 + 1].ddx = data[i];
            this.poses[i / 2 + 1].ddy = data[i + 1];

            this.table.updateFromCanvas(this.poses[i / 2] + 1);
        }
    }


    convert(pose, i) {
        var newPose = new Pose2d();
        newPose.x = in_to_px(pose.x, axes.x);
        newPose.y = in_to_px(pose.y, axes.y);
        newPose.dx = in_to_px(pose.dx);
        newPose.dy = in_to_px(pose.dy);
        newPose.ddx = in_to_px(pose.ddx);
        newPose.ddy = in_to_px(pose.ddy);
        newPose.poseIndex = i;

        return newPose;
    }

    setPoses(tablePoses) {
        this.poses = tablePoses.map(this.convert);

        this.posPointsData = this.poses.map(pose => {
            return {
                cx: pose.x,
                cy: pose.y,
                width: this.robotLength,
                height: this.robotWidth,
                poseIndex: pose.poseIndex,
                heading: 180 / Math.PI * pose.heading
            };
        });

        this.velPointsData = this.poses.map(pose => {
            return {
                r: 5,
                x: pose.x + pose.dx,
                y: pose.y + pose.dy,
                poseIndex: pose.poseIndex
            };
        });

        this.updatePosPoints();
        this.updateLines();
        this.updateVelPoints();
    }

    startDragImg() {
        this.currPose = new Pose2d([d3.event.x, d3.event.y, heading_weight, 0, 0, 0]);
        this.currPose.poseIndex = this.poses.length;
        this.addPose(this.currPose);
        this.table.addPose(this.currPose);

        this.currPos = this.posPoints.filter(posPointData =>
            posPointData.poseIndex === this.currPose.poseIndex);
        this.currLine = this.lines.filter(pose =>
            pose.poseIndex == this.currPose.poseIndex);
        this.currVel = this.velPoints.filter(velPointData =>
            velPointData.poseIndex == this.currPose.poseIndex);

        this.currVel.classed(this.activeClass, true);
    }

    endDragImg() {
        var dx = d3.event.x - this.currPose.x;
        var dy = d3.event.y - this.currPose.y;

        var heading = Math.atan2(dy, dx);

        var [vx, vy] = this.velFromOrigin(this.currPose.x, this.currPose.y, heading, heading_weight);

        this.currPose.dx = vx - this.currPose.x;
        this.currPose.dy = vy - this.currPose.y;

        this.currPos.datum(r => {
            r.heading = 180 / Math.PI * heading;
            return r;
        });

        this.currLine.datum(this.currPose);

        this.currVel.datum(r => {
            r.x = this.currPose.x + this.currPose.dx;
            r.y = this.currPose.y + this.currPose.dy;
            return r;
        });

        this.currVel.classed(this.activeClass, false);

        if (this.table) {
            this.table.updateFromCanvas(this.currPose);
        }
    }

    draggedImg() {
        if (!d3.event.dx && !d3.event.dy) return;

        var dx = d3.event.x - this.currPose.x;
        var dy = d3.event.y - this.currPose.y;

        var heading = Math.atan2(dy, dx);

        var [vx, vy] = this.velFromOrigin(this.currPose.x, this.currPose.y, heading, heading_weight);

        this.currPos
            .attr('transform', r => 'rotate(' + (180 / Math.PI * heading) + ' ' + r.cx + ' ' + r.cy + ')');
        this.currLine
            .attr('x2', vx)
            .attr('y2', vy);
        this.currVel
            .attr('cx', vx)
            .attr('cy', vy);

        this.currPose.dx = vx - this.currPose.x;
        this.currPose.dy = vy - this.currPose.y;


        this.updatePath();
    }

    startDragVel(d) {
        this.currPos = this.posPoints.filter(posPointData =>
            posPointData.poseIndex === d.poseIndex);
        this.currLine = this.lines.filter(pose =>
            pose.poseIndex == d.poseIndex);
        this.currVel = this.velPoints.filter(velPointData =>
            velPointData.poseIndex == d.poseIndex);

        this.currPose = this.poses[d.poseIndex];

        this.currVel.classed(this.activeClass, true);
    }

    endDragVel(d) {
        this.currPose.dx = d.x - this.currPose.x;
        this.currPose.dy = d.y - this.currPose.y;

        var heading = 180 / Math.PI * this.currPose.heading;

        this.currPos.datum(r => {
            r.heading = heading;
            return r;
        });

        this.currVel.classed(this.activeClass, false);

        if (this.table) {
            this.table.updateFromCanvas(this.currPose);
        }
    }

    velFromOrigin(x, y, heading, dist) {
        return [x + Math.cos(heading) * dist, y + Math.sin(heading) * dist];
    }

    draggedVel(d) {
        if (!d3.event.dx && !d3.event.dy) return;

        var heading = Math.atan2(d3.event.y - this.currPose.y, d3.event.x - this.currPose.x);

        var [vx, vy] = this.velFromOrigin(this.currPose.x, this.currPose.y, heading, heading_weight);

        d.x = vx;
        d.y = vy;

        this.currPos
            .attr('transform', r => 'rotate(' + (180 / Math.PI * heading) + ' ' + r.cx + ' ' + r.cy + ')');
        this.currLine
            .attr('x2', d.x)
            .attr('y2', d.y);
        this.currVel
            .attr('cx', d.x)
            .attr('cy', d.y);

        this.currPose.dx = d.x - this.currPose.x;
        this.currPose.dy = d.y - this.currPose.y;
        this.updatePath();
    }

    startDragPos(d) {
        this.currPos = this.posPoints.filter(posPointData =>
            posPointData.poseIndex === d.poseIndex);
        this.currLine = this.lines.filter(pose =>
            pose.poseIndex == d.poseIndex);
        this.currVel = this.velPoints.filter(velPointData =>
            velPointData.poseIndex == d.poseIndex);

        this.currPose = this.poses[d.poseIndex];

        this.currPos.classed(this.activeClass, true);
    }

    endDragPos(d) {
        this.currPose.x = d.cx;
        this.currPose.y = d.cy;

        this.currVel
            .datum(c => {
                c.x = this.currPose.x + this.currPose.dx;
                c.y = this.currPose.y + this.currPose.dy;
                return c;
            });

        this.currPos.classed(this.activeClass, false);

        if (this.table) {
            this.table.updateFromCanvas(this.currPose);
        }
    }

    draggedPos(d) {
        if (!d3.event.dx && !d3.event.dy) return;
        d.cx = d3.event.x;
        d.cy = d3.event.y;

        var heading = 180 / Math.PI * this.currPose.heading;

        this.currPos
            .attr('x', r => d.cx - r.width / 2)
            .attr('y', r => d.cy - r.height / 2)
            .attr('transform', r => 'rotate(' + heading + ' ' + d.cx + ' ' + d.cy + ')');
        this.currLine
            .attr('x1', d.cx)
            .attr('y1', d.cy)
            .attr('x2', d.cx + this.currPose.dx)
            .attr('y2', d.cy + this.currPose.dy);
        this.currVel
            .attr('cx', d.cx + this.currPose.dx)
            .attr('cy', d.cy + this.currPose.dy);

        this.currPose.x = d.cx;
        this.currPose.y = d.cy;
        this.updatePath();
    }

    addPose(pose) {
        this.poses.push(pose);

        this.posPointsData.push({
            cx: pose.x,
            cy: pose.y,
            width: this.robotLength,
            height: this.robotWidth,
            poseIndex: pose.poseIndex,
            heading: 180 / Math.PI * pose.heading
        });

        this.velPointsData.push({
            r: 5,
            x: pose.x + pose.dx,
            y: pose.y + pose.dy,
            poseIndex: pose.poseIndex
        });

        this.updatePosPoints();
        this.updateLines();
        this.updateVelPoints();
    }

    updatePosPoints(posData = this.posPointsData) {
        this.posPoints = this.posPoints
            .data(posData);

        this.posPoints.exit().remove();

        this.posPoints = this.posPoints
            .enter()
            .append('rect')
            .merge(this.posPoints)
            .attr('x', r => r.cx - r.width / 2)
            .attr('y', r => r.cy - r.height / 2)
            .attr('width', r => r.width)
            .attr('height', r => r.height)
            .style('stroke-width', 2)
            .style('fill-opacity', 0.3)
            .attr('transform', r => 'rotate(' + r.heading + ' ' + r.cx + ' ' + r.cy + ')')
            .call(this.posDragBehavior);
    }

    updateLines(lineData = this.poses) {
        this.lines = this.lines
            .data(lineData);

        this.lines.exit().remove();

        this.lines = this.lines
            .enter()
            .append('line')
            .merge(this.lines)
            .attr('x1', d => d.x)
            .attr('y1', d => d.y)
            .attr('x2', d => d.x + d.dx)
            .attr('y2', d => d.y + d.dy);
    }

    updateVelPoints(velData = this.velPointsData) {
        this.velPoints = this.velPoints
            .data(velData);

        this.velPoints.exit().remove();

        this.velPoints = this.velPoints
            .enter()
            .append('circle')
            .merge(this.velPoints)
            .attr('r', d => d.r)
            .attr('cx', d => d.x)
            .attr('cy', d => d.y)
            .call(this.velDragBehavior);
    }
}

export default Field;
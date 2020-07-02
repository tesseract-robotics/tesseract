// Copyright 2019 Wason Technology, LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//    http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: John Wason (wason@wasontech.com)
// Date: 12/10/2019
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
class TesseractViewer {
    constructor(canvasElement) {
        this._scene_etag = null;
        this._trajectory_etag = null;
        // Create canvas and engine.
        this._canvas = document.getElementById(canvasElement);
        this._engine = new BABYLON.Engine(this._canvas, true);
    }
    createScene() {
        return __awaiter(this, void 0, void 0, function* () {
            // Create the scene space
            this._scene = new BABYLON.Scene(this._engine);
            //scene.clearColor = new BABYLON.Color3(.4, .4, .4);
            this._scene.useRightHandedSystem = true;
            this._environment = this._scene.createDefaultEnvironment({ enableGroundShadow: true, groundYBias: 0 });
            this._environment.setMainColor(BABYLON.Color3.FromHexString("#74b9ff"));
            // Add a camera to the scene and attach it to the canvas
            this._camera = new BABYLON.ArcRotateCamera("Camera", Math.PI / 2, Math.PI / 2, 2, new BABYLON.Vector3(0, 1, 0), this._scene);
            this._camera.attachControl(this._canvas, true);
            this._camera.setPosition(new BABYLON.Vector3(2.5, 1.5, -1));
            // Add lights to the scene
            this._light = new BABYLON.HemisphericLight("light1", new BABYLON.Vector3(0, -1, 0), this._scene);
            this._root = new BABYLON.TransformNode("root0");
            this._root.rotation.x = -1.5707963267948966;
            yield this.updateScene();
            console.log("Loaded!");
            this._scene.transformNodes.forEach(function (tf) {
                //this._addAxis(tf, 0.5);
                //console.log(tf)
            });
            //console.log(this._scene.transformNodes);
            this._scene.meshes.forEach(function (m) {
                try {
                    m.createNormals(true);
                }
                catch (e) {
                    console.log(e);
                }
                //m.parent=root;
                //addAxis(tf, 0.5);
            });
            yield this.enableVR();
            let _this = this;
            setTimeout(() => _this.updateTrajectory(), 2000);
            //this._scene.debugLayer.show();
        });
    }
    doRender() {
        // Run the render loop.
        this._engine.runRenderLoop(() => {
            this._scene.render();
        });
        // The canvas/window resize event handler.
        window.addEventListener('resize', () => {
            this._engine.resize();
        });
    }
    addAxis(parent, size) {
        var axisX = BABYLON.Mesh.CreateLines("axisX", [
            BABYLON.Vector3.Zero(), new BABYLON.Vector3(size, 0, 0), new BABYLON.Vector3(size * 0.95, 0.05 * size, 0),
            new BABYLON.Vector3(size, 0, 0), new BABYLON.Vector3(size * 0.95, -0.05 * size, 0)
        ], this._scene);
        axisX.color = new BABYLON.Color3(1, 0, 0);
        axisX.parent = parent;
        //var xChar = makeTextPlane("X", "red", size / 10);
        //xChar.position = new BABYLON.Vector3(0.9 * size, -0.05 * size, 0);
        var axisY = BABYLON.Mesh.CreateLines("axisY", [
            BABYLON.Vector3.Zero(), new BABYLON.Vector3(0, size, 0), new BABYLON.Vector3(-0.05 * size, size * 0.95, 0),
            new BABYLON.Vector3(0, size, 0), new BABYLON.Vector3(0.05 * size, size * 0.95, 0)
        ], this._scene);
        axisY.color = new BABYLON.Color3(0, 1, 0);
        axisY.parent = parent;
        //var yChar = makeTextPlane("Y", "green", size / 10);
        //yChar.position = new BABYLON.Vector3(0, 0.9 * size, -0.05 * size);
        var axisZ = BABYLON.Mesh.CreateLines("axisZ", [
            BABYLON.Vector3.Zero(), new BABYLON.Vector3(0, 0, size), new BABYLON.Vector3(0, -0.05 * size, size * 0.95),
            new BABYLON.Vector3(0, 0, size), new BABYLON.Vector3(0, 0.05 * size, size * 0.95)
        ], this._scene);
        axisZ.color = new BABYLON.Color3(0, 0, 1);
        axisZ.parent = parent;
        //var zChar = makeTextPlane("Z", "blue", size / 10);
        //zChar.position = new BABYLON.Vector3(0, 0.05 * size, 0.9 * size);
    }
    enableVR() {
        return __awaiter(this, void 0, void 0, function* () {
            // Enable VR
            var ground = BABYLON.MeshBuilder.CreateGround("ground", { width: 10, height: 10 }, this._scene);
            ground.material = new BABYLON.GridMaterial("mat", this._scene);
            const xrHelper = yield this._scene.createDefaultXRExperienceAsync({
                // define floor meshes
                floorMeshes: [ground]
            });
            ground.visibility = 0.1;
            //vrHelper.enableTeleportation({floorMeshes: [environment.ground]});
        });
    }
    updateScene() {
        return __awaiter(this, void 0, void 0, function* () {
            let fetch_res;
            try {
                fetch_res = yield fetch("tesseract_scene.babylon", { method: "HEAD" });
            }
            catch (_a) {
                let _this = this;
                setTimeout(() => _this.updateScene(), 1000);
                return;
            }
            let etag = fetch_res.headers.get('etag');
            if (etag !== null) {
                if (this._scene_etag !== null) {
                    if (this._scene_etag != etag) {
                        location.reload();
                        return;
                    }
                    else {
                        let _this = this;
                        setTimeout(() => _this.updateScene(), 1000);
                        return;
                    }
                }
            }
            yield BABYLON.SceneLoader.AppendAsync("./", "tesseract_scene.babylon", this._scene);
            if (etag !== null) {
                this._scene_etag = etag;
                let _this = this;
                setTimeout(() => _this.updateScene(), 1000);
            }
        });
    }
    updateTrajectory() {
        return __awaiter(this, void 0, void 0, function* () {
            let fetch_res;
            let _this = this;
            try {
                fetch_res = yield fetch("tesseract_trajectory.json", { method: "HEAD" });
            }
            catch (_a) {
                setTimeout(() => _this.updateTrajectory(), 1000);
                return;
            }
            if (!fetch_res.ok) {
                setTimeout(() => _this.updateTrajectory(), 1000);
                return;
            }
            let etag = fetch_res.headers.get('etag');
            if (etag == null || this._trajectory_etag == etag) {
                console.log("No updated trajectory");
                setTimeout(() => _this.updateTrajectory(), 1000);
                return;
            }
            try {
                if (this._joint_trajectory !== null) {
                    this._joint_trajectory.stop();
                    this._joint_trajectory = null;
                }
            }
            catch (_b) { }
            try {
                let trajectory_response = yield fetch("./tesseract_trajectory.json");
                let trajectory_json = yield trajectory_response.json();
                this._joint_trajectory = JointTrajectoryAnimation.Parse(trajectory_json, this._scene);
                this._joint_trajectory.start();
            }
            catch (e) {
                console.log("Trajectory not available");
                console.log(e);
            }
            if (etag !== null) {
                this._trajectory_etag = etag;
                setTimeout(() => _this.updateTrajectory(), 1000);
            }
        });
    }
}
class JointTrajectoryAnimation {
    constructor(scene, joint_names, trajectory, use_time, loop_time) {
        this._timerid = 0;
        if (joint_names.length == 0) {
            throw new Error("joint_names must not be zero count");
        }
        if (trajectory.length == 0) {
            throw new Error("trajectory must not be zero count");
        }
        this._max_time = -1;
        trajectory.forEach((t) => {
            if (use_time) {
                if (t.length - 1 != joint_names.length) {
                    throw new Error("Trajectory waypoints must have same count as joint_names");
                }
                let waypoint_time = t.slice(-1)[0];
                if (this._max_time >= waypoint_time) {
                    throw new Error("Trajectory waypoint time must me monotonically increasing");
                }
                this._max_time = waypoint_time;
            }
            else {
                if (t.length != joint_names.length) {
                    throw new Error("Trajectory waypoints must have same count as joint_names");
                }
            }
        });
        this._joint_names = joint_names;
        this._trajectory = trajectory;
        this._use_time = use_time;
        this._loop_time = loop_time;
        this._scene = scene;
        this.findJoints();
    }
    findJoints() {
        let joints = new Map();
        let axes = new Map();
        let type = new Map();
        this._joint_names.forEach((joint_name) => {
            let tf = this._scene.getTransformNodeByName("joint_" + joint_name);
            let metadata = tf.metadata;
            if (metadata.hasOwnProperty("tesseract_joint")
                && metadata.tesseract_joint.hasOwnProperty("axis")) {
                joints.set(joint_name, tf);
                let axis_array = tf.metadata.tesseract_joint.axis;
                axes.set(joint_name, new BABYLON.Vector3(axis_array[0], axis_array[1], axis_array[2]));
                type.set(joint_name, tf.metadata.tesseract_joint.type);
            }
        });
        this._joints = joints;
        this._joint_axes = axes;
        this._joint_type = type;
    }
    resetJointPos() {
        this._joints.forEach((tf) => {
            tf.position = new BABYLON.Vector3(0, 0, 0);
            tf.rotationQuaternion = new BABYLON.Quaternion(0, 0, 0, 1);
        });
    }
    getMaxTime() {
        if (this._use_time) {
            return this._max_time;
        }
        else {
            return this._loop_time;
        }
    }
    setTrajectoryTime(t) {
        let joint_n = this._joint_names.length;
        let n = this._trajectory.length;
        let times = [];
        for (let i = 0; i < n; i++) {
            if (this._use_time) {
                times.push(this._trajectory[i][joint_n]);
            }
            else {
                times.push(i * (this._loop_time / n));
            }
        }
        let joint_pos = null;
        for (let i = 0; i < n - 1; i++) {
            if (times[i] == t) {
                joint_pos = this._trajectory[i].slice(0, joint_n);
                break;
            }
            if (times[i] < t) {
                let joint_pos1 = this._trajectory[i].slice(0, joint_n);
                let joint_pos2 = this._trajectory[i + 1].slice(0, joint_n);
                let t1 = times[i];
                let t2 = times[i + 1];
                joint_pos = [];
                for (let j = 0; j < joint_n; j++) {
                    joint_pos.push(joint_pos1[j] + ((joint_pos2[j] - joint_pos1[j]) / (t2 - t1)) * (t - t1));
                }
            }
        }
        if (joint_pos === null) {
            joint_pos = this._trajectory.slice(-1)[0].slice(0, joint_n);
        }
        for (let i = 0; i < joint_n; i++) {
            let joint_name = this._joint_names[i];
            let joint = this._joints.get(joint_name);
            let axes = this._joint_axes.get(joint_name);
            let type = this._joint_type.get(joint_name);
            if (type == 2) {
                joint.position = axes.scale(joint_pos[i]);
            }
            else {
                joint.rotationQuaternion = new BABYLON.Quaternion(0, 0, 0, 1);
                joint.rotate(axes, joint_pos[i], BABYLON.Space.LOCAL);
            }
        }
    }
    start() {
        if (this._timerid != 0) {
            return;
        }
        this._t0 = new Date().getTime() / 1000;
        var _this = this;
        this._timerid = setInterval(() => _this.intervalCallback(), 50);
    }
    stop() {
        if (this._timerid == 0) {
            return;
        }
        clearInterval(this._timerid);
        this._timerid = 0;
    }
    intervalCallback() {
        let max_t = this.getMaxTime();
        let t_total = new Date().getTime() / 1000 - this._t0;
        let t = t_total % max_t;
        this.setTrajectoryTime(t);
    }
    static Parse(parsedTrajectory, scene) {
        let trajectory = new JointTrajectoryAnimation(scene, parsedTrajectory.joint_names, parsedTrajectory.trajectory, parsedTrajectory.use_time, parsedTrajectory.loop_time);
        return trajectory;
    }
}
window.addEventListener('DOMContentLoaded', function () {
    return __awaiter(this, void 0, void 0, function* () {
        // Create the game using the 'renderCanvas'.
        let viewer = new TesseractViewer('renderCanvas');
        // Create the scene.
        yield viewer.createScene();
        // Start render loop.
        viewer.doRender();
    });
});


function findJointByName(nodeName) {
    for (var i = 0; i < 87; i++) {
        if (kinematics.jointMap[i].node.name == nodeName) {
            return i;
        }
    }
}

function cameraProject(point3d, width, height) {
    var point2d = point3d.project(camera);
    var p = new THREE.Vector2;
    p.x =  (point2d.x + 1) / 2 * width;
    p.y = -(point2d.y - 1) / 2 * height;
    return p;
}

function localToWorldSceneGraph(point, link) {
    var drag_point_local = link.worldToLocal(point);
    var current_link = link;
    while (current_link.parent) {
        //console.log(current_link.name);
        drag_point_local.applyMatrix4(current_link.matrix);
        current_link = current_link.parent;
    }
    return point;
}

// function projectPoint() {
//     var arm = dae.getObjectByName(arm_link_name);
//     raycaster.setFromCamera( cursor_normalized, camera );
//     var intersects = raycaster.intersectObject( arm, true );
//     if (intersects.length > 0) {
//         var link = intersects[0].object.parent;
//         var point = intersects[0].point.clone();
//         point = localToWorldSceneGraph(point.clone(), link);
//         projected_point = cameraProject(point, renderer.domElement.width, renderer.domElement.height);
//     }
// }

function transformToMatrix(transform) {
    var m1 = new THREE.Matrix4();
    switch ( transform.type ) {
        case 'matrix':
            return transform.obj;
        case 'translate':
            return m1.makeTranslation( transform.obj.x, transform.obj.y, transform.obj.z );
        case 'rotate':
            return m1.makeRotationAxis( transform.obj, transform.angle );
    }
}

N_JOINTS = 4;
function initSolverSceneGraph(link, arm) {
    // Called once when the DAE is finished loading
    //   this function is idempotent - running it twice does nothing.
    //   this is just a hack because I can't find the right place to
    //   call this after DAE loads, so I call it repeatedly
    if (solver.getNumTransforms() == 0) {
        console.log('initializing solver scene graph in initSolverSceneGraph');
        // first enumerate the links connecting 'link' to 'arm'
        var links = [];
        var current_link = link;
        while (current_link.parent) {
            console.log(current_link.name);
            links.push(current_link);
            if (current_link.name == arm.name) {
                break;
            }
            current_link = current_link.parent;
        }
        console.log(links);
        // then, starting with 'arm', add links to the solver until N_JOINTS joints have been added
        // arm to world
        solver.addStaticTransform(arm.parent.matrixWorld.elements);
        var num_joints = 0;
        for (var i = links.length - 1; i >= 0; i--) {
            var putative_joint = findJointByName(links[i].name);
            console.log('putative_joint = ' + putative_joint);
            if (putative_joint == null || kinematics.jointMap[putative_joint].joint.static || num_joints >= N_JOINTS) {
                console.log('adding static transform for ' + links[i].name);
                solver.addStaticTransform(links[i].matrix.elements);
                linkToIndex[links[i].name] = solver.getNumTransforms() - 1;
            } else {
                console.log('adding joint for ' + links[i].name);
                 // Set static transforms
                var transforms = kinematics.jointMap[putative_joint].transforms;
                var m01 = transformToMatrix(transforms[0]).multiply(transformToMatrix(transforms[1]));
                var m34 = transformToMatrix(transforms[3]).multiply(transformToMatrix(transforms[4]));
                solver.addStaticTransform(m01.elements);
                console.log('kinematics.jointMap[putative_joint].joint.limits.min = ' + kinematics.jointMap[putative_joint].joint.limits.min);
                console.log('kinematics.jointMap[putative_joint].joint.limits.max= ' + kinematics.jointMap[putative_joint].joint.limits.max);
                solver.addJointTransform(kinematics.jointMap[putative_joint].joint.limits.min,
                                         kinematics.jointMap[putative_joint].joint.limits.max);
                solver.addStaticTransform(m34.elements);
                linkToIndex[links[i].name] = solver.getNumTransforms() - 1;
                num_joints++;
            }
        }
    }
}

function initIK(point, link, arm) {
    // Called once at the beginning of a drag in order to reset the IK optimization
    initSolverSceneGraph(dae.getObjectByName('l_gripper_palm_link'), arm);

    // Set drag point
    drag_point = point;
    var nearest_link = link;
    while (!linkToIndex[nearest_link.name]) {
        nearest_link = nearest_link.parent;
    }
    var drag_point_local = nearest_link.worldToLocal(drag_point.clone());
    solver.drag_point = [drag_point_local.x, drag_point_local.y, drag_point_local.z];
    solver.setStartTransformIndex( linkToIndex[nearest_link.name] );

    // Visualize the drag point
    drag_point_visual.position.set(drag_point_local.x, drag_point_local.y, drag_point_local.z);
    if (drag_point_visual.parent) {
        drag_point_visual.parent.remove(drag_point_visual);
    }
    nearest_link.add(drag_point_visual);

    // Set screen size
    solver.setDims([renderer.domElement.width, renderer.domElement.height]);

    // Set camera matrix
    var a = new THREE.Matrix4;
    a.multiplyMatrices(camera.projectionMatrix, a.getInverse(camera.matrixWorld));
    solver.setCameraMatrix(a.elements);
}

function solveIK() {
    solver.screen_point = [cursor.x, cursor.y];
    solver.timeSolve(0.1);
    for (var i = 0; i < N_JOINTS; i++) {
        kinematics.setJointValue(arm_joint_idx + i, solver.getJointValue(i));
    }
}

function setDragging(dragging) {
    dragging_object = dragging;
    drag_point_visual.visible = dragging;
    // controls.enabled = !dragging;
    // controls.enableRotate = !dragging;
}

function handleMouseDown() {
    // update the picking ray with the camera and mouse position
    raycaster.setFromCamera( cursor_normalized, camera );
    // calculate objects intersecting the picking ray
    var arm = dae.getObjectByName(arm_link_name);
    var intersects = raycaster.intersectObject( arm, true );
    if (intersects.length > 0) {
        initIK(intersects[0].point, intersects[0].object.parent, arm);
        setDragging(true);
    } else {
        setDragging(false);
    }
}

function handleMouseUp() {
    setDragging(false);
}

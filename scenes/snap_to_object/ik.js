var IK = (function () {
  // the variable IK will point to the module object
  module = {};

  var use2d = false;

  var linkToIndex = {};
  var dragging_object = false;
  var drag_point_attached = false;

  var arm_link_name = 'l_shoulder_pan_link';
  var arm;
  var target_object;

  var raycaster = new THREE.Raycaster();
  var cursor = new THREE.Vector2( -200, -200 );
  var cursor_normalized = new THREE.Vector2( -200, -200 );
  var drag_point = new THREE.Vector3( 0, 0, 0);
  var projected_point = new THREE.Vector2( -200, -200 );
  var snap_guide;

  // Create the drag point dot
  var material = new THREE.MeshBasicMaterial( { color: 0x000000 } );
  var geometry = new THREE.SphereGeometry(0.01);
  var drag_point_visual = new THREE.Mesh( geometry, material );
  drag_point_visual.visible = false;

/*
  // Snap guide
  var material = new THREE.LineBasicMaterial( { color: 0x333333, linewidth: 1 } );
  var geometry = new THREE.Geometry();
  geometry.vertices.push(new THREE.Vector3(0,0,0));
  geometry.vertices.push(new THREE.Vector3(1,0,0));
  snap_guide = new THREE.Line(geometry, material);
  snap_guide.visible = false;
  dae.getObjectByName('l_gripper_l_finger_tip_link').add(snap_guide);
*/

  var arm_joint_idx;
  var solver;
  var camera;
  var renderer;
  var robot;
  var kinematics;
  var marker;

  var MARKER_OFFSET = 0.75;

  module.init = function (options) {
    var viewer = options.viewer;
    robot = options.robot;
    kinematics = options.kinematics;
    marker = options.marker;

    camera = viewer.camera;
    renderer = viewer.renderer;

    arm_joint_idx = findJointByName(arm_link_name);
    arm = robot.getObjectByName(arm_link_name);

    solver = new Module.IKSolver;
    module.solver = solver;
    initJoints();
    if (use2d) {
        // HACK: remove marker
        viewer.selectableObjects.children.splice(0,1);
        solver.addTargetPoint2D();
    } else {
        solver.addTargetPoint3D();
        solver.addTargetPoint3D();
        solver.addTargetPoint3D();
        // move marker
        setTimeout(function() {
            var pos = robot.getObjectByName('l_gripper_palm_link').getWorldPosition();
            pos.x += MARKER_OFFSET;
            console.log(pos);
            marker.position.copy(pos);
            //marker.addEventListener('mousedown', handleMouseDown);
            //marker.addEventListener('mouseup', handleMouseUp);
        }, 500);
    }

  // Snap guide
  var material = new THREE.LineBasicMaterial( { color: 0x333333, linewidth: 1 } );
  var geometry = new THREE.Geometry();
  geometry.vertices.push(new THREE.Vector3(0,0,0));
  geometry.vertices.push(new THREE.Vector3(1,0,0));
  snap_guide = new THREE.Line(geometry, material);
  snap_guide.visible = false;
  robot.getObjectByName('l_gripper_l_finger_tip_link').add(snap_guide);

  // Add target object 
  var material = new THREE.MeshPhongMaterial( { color: 0x666666 } );
  var geometry = new THREE.SphereGeometry(0.5); 
  target_object = new THREE.Mesh( geometry, material );
  viewer.scene.add(target_object);
  target_object.position.set(6.0, 0, 5.0);

  // Add table 
  var material = new THREE.MeshPhongMaterial( { color: 0x666666 } );
  var geometry = new THREE.BoxGeometry(3.0, 5.0, 3.5); 
  target_object = new THREE.Mesh( geometry, material );
  viewer.scene.add(target_object);
  target_object.position.set(6.0, 0.0, 2.25);


  // Add solver callback
  viewer.addDrawCallback(function () {
      if (dragging_object) {
        solveIK();
      }
    });

    // IK callbacks
    document.addEventListener('mousedown', handleMouseDown);
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousemove', function(event) {
        // calculate mouse position in normalized device coordinates
        // (-1 to +1) for both components
        cursor_normalized.x = ( event.x / window.innerWidth ) * 2 - 1;
        cursor_normalized.y = - ( event.y / window.innerHeight ) * 2 + 1;
        cursor.x = event.x;
        cursor.y = event.y;
    });

    // ROS3D callbacks
    /*
    dae.addEventListener('mouseover', stopPropagation);
    dae.addEventListener('mousedown', stopPropagation);
    dae.addEventListener('mouseup', stopPropagation);
    */
  }

  function stopPropagation(event) {
    event.stopPropagation();
  }

  function snapControlObject(obj) {
      if (Math.abs(obj.position.y) < 0.5) {
          obj.position.y = 0.0;
          snap_guide.visible = true;
      } else {
          snap_guide.visible = false;
      }
  }

  function getControlPoints(obj) {
      var pos = obj.getWorldPosition().clone();
      var x_offset = new THREE.Vector3(1, 0, 0);
      var y_offset = new THREE.Vector3(0, 1, 0);
      var points = [
        pos.clone(),
        pos.clone().add(x_offset.applyQuaternion(obj.quaternion)),
        pos.clone().add(y_offset.applyQuaternion(obj.quaternion))
      ]
      return points;
  }

  function findJointByName(nodeName) {
    console.log(kinematics);
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

  N_JOINTS = 7;
  function initSolverSceneGraph() {
      var link = robot.getObjectByName('l_gripper_palm_link');
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

  // Called once at the beginning of a drag in order to reset the IK optimization
  function attachDragPointToLink(pt_idx, point, link) {
      initSolverSceneGraph();

      // Set 2D drag point
      drag_point = point;
      var nearest_link = link;
      while (!linkToIndex[nearest_link.name]) {
          nearest_link = nearest_link.parent;
      }
      if (use2d) {
          var drag_point_local = nearest_link.worldToLocal(drag_point.clone());
          solver.setDragPoint2D(pt_idx, [drag_point_local.x, drag_point_local.y, drag_point_local.z]);
          // Visualize the drag point
          drag_point_visual.position.set(drag_point_local.x, drag_point_local.y, drag_point_local.z);
          if (drag_point_visual.parent) {
              drag_point_visual.parent.remove(drag_point_visual);
          }
          nearest_link.add(drag_point_visual);
      } else {
          var drag_point_local = nearest_link.worldToLocal(drag_point.clone());
          solver.setDragPoint3D(pt_idx, [drag_point_local.x, drag_point_local.y, drag_point_local.z]);
      }
      solver.setStartTransformIndex( linkToIndex[nearest_link.name] );

      // Set screen size
      solver.setDims([renderer.domElement.width, renderer.domElement.height]);

      // Set camera matrix
      var a = new THREE.Matrix4;
      a.multiplyMatrices(camera.projectionMatrix, a.getInverse(camera.matrixWorld));
      solver.setCameraMatrix(a.elements);
  }

  function initJoints() {
    var jointVals = [
        57.40083954304086,
        3.897952346576082,
        114.94638942664692,
        -110.95863793571972,
        -153.2146794128689,
        -59.56016827522665,
        64.68627613717995,
    ];
    for (var i = 0; i < N_JOINTS; i++) {
        solver.setJointValue(i, jointVals[i]);
        kinematics.setJointValue(arm_joint_idx + i, jointVals[i]);
    }
  }

  function solveIK() {
      if (use2d) {
          solver.setTargetPoint2D(0, [cursor.x, cursor.y]);
      } else {
          snapControlObject(marker);
          var controlPoints = getControlPoints(marker);
          for (var i = 0; i < controlPoints.length; i++) {
              solver.setTargetPoint3D(i, [controlPoints[i].x, controlPoints[i].y, controlPoints[i].z]);
          }
      }
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

  function handleMouseDown(event) {
      if (!use2d) {
          if (!drag_point_attached) {
              var controlPoints = getControlPoints(marker);
              for (var i = 0; i < controlPoints.length; i++) {
                  attachDragPointToLink(i, controlPoints[i], robot.getObjectByName('l_gripper_palm_link'));
              }
              drag_point_attached = true;
          }
          setDragging(true);
      } else {
          // update the picking ray with the camera and mouse position
          raycaster.setFromCamera( cursor_normalized, camera );
          // calculate objects intersecting the picking ray
          var intersects = raycaster.intersectObject( arm, true );
          if (intersects.length > 0) {
              attachDragPointToLink(0, intersects[0].point, intersects[0].object.parent);
              setDragging(true);
          } else {
              setDragging(false);
          }
      }
  }

  function handleMouseUp(event) {
      setDragging(false);
  }

  // Export a few variables to help debugging
  return module;
})();

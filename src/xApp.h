#pragma once

#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "aiMesh.h"
#include "aiScene.h"
#include "ofxBullet.h"
#include "skeleton.h"

class xApp : public ofBaseApp{

        public:
        ofxBulletWorldRigid world;
        ofxBulletBox        ground;
        vector<ofxBulletBox*> box;
        vector<ofxBulletBox*> bounds;

        ofxBulletCustomShape* boundsShape;
        float boundsWidth = 1300.;

        RiggedModel model;
        ofVboMesh body;
        ofLight light;
        bool wireframe;

        void bulletPosAndRotBone() {
            Pose pose = model.getPose();
            int counter=0;
            for(Pose::iterator i = pose.begin(); i != pose.end(); i++) {
                aiMatrix4x4& cur = i->second;

                ofQuaternion quat;
                ofMatrix4x4 mat;

                ofVec3f axis = box[counter]->getPosition()*150;
                axis.normalize();
                quat.makeRotate(box[counter]->getRotationAngle()*50, axis);
                quat.get(mat);

                cur.a1 = mat(0, 0);
                cur.a2 = mat(0, 1);
                cur.a3 = mat(0, 2);
                cur.b1 = mat(1, 0);
                cur.b2 = mat(1, 1);
                cur.b3 = mat(1, 2);
                cur.c1 = mat(2, 0);
                cur.c2 = mat(2, 1);
                cur.c3 = mat(2, 2);
                mat.makeTranslationMatrix(axis);
                counter++;
            }
            model.pose(pose);
        }

        ofCamera cam;

        //--------------------------------------------------------------
        void setup(){
            cam.setPosition(ofVec3f(0, -690.f, -1000.f));
            cam.lookAt(ofVec3f(0, 0, 0), ofVec3f(0, -1, 0));

            world.setup();
            world.enableGrabbing();
            world.setCamera(&cam);
            world.setGravity( ofVec3f(0, 400, 0) );

            ofVec3f startLoc;
            ofPoint dimens;

            float hwidth = boundsWidth*.5;
            float depth = 2.;
            float hdepth = depth*.5;
            boundsShape = new ofxBulletCustomShape();
            boundsShape->create(world.world, ofVec3f(0, 0, 0), 10.);

            for(int i = 0; i < 6; i++) {
                    bounds.push_back( new ofxBulletBox() );
                    if(i == 0) { // ground //
                            startLoc.set( 0., hwidth+hdepth, 0. );
                            dimens.set(boundsWidth, depth, boundsWidth);
                    } else if (i == 1) { // back wall //
                            startLoc.set(0, 0, hwidth+hdepth);
                            dimens.set(boundsWidth, boundsWidth, depth);
                    } else if (i == 2) { // right wall //
                            startLoc.set(hwidth+hdepth, 0, 0.);
                            dimens.set(depth, boundsWidth, boundsWidth);
                    } else if (i == 3) { // left wall //
                            startLoc.set(-hwidth-hdepth, 0, 0.);
                            dimens.set(depth, boundsWidth, boundsWidth);
                    } else if (i == 4) { // ceiling //
                            startLoc.set(0, -hwidth-hdepth, 0.);
                            dimens.set(boundsWidth, depth, boundsWidth);
                    } else if (i == 5) { // front wall //
                            startLoc.set(0, 0, -hwidth-hdepth);
                            dimens.set(boundsWidth, boundsWidth, depth);
                    }
                    btBoxShape* boxShape = ofBtGetBoxCollisionShape( dimens.x, dimens.y, dimens.z );
                    boundsShape->addShape( boxShape, startLoc );

                    bounds[i]->create( world.world, startLoc, 0., dimens.x, dimens.y, dimens.z );
                    bounds[i]->setProperties(.25, .95);
                    bounds[i]->add();
            }

            model.loadModel("avatar.dae");

            model.calculateSkeleton();
            cout<<"size-skeleton::"<<model.skeleton.size()<<endl;
            for(int i = 0; i < model.skeleton.size(); i++ ){
                ofMesh face1;
                face1.addVertices(&model.skeleton[i].mesh.getVertices()[0],4);
                ofxBulletBox* xbox = new ofxBulletBox();
                xbox->create(world.world, ofVec3f(face1.getCentroid().x,face1.getCentroid().y-400,face1.getCentroid().z-200), 1.0, 25.5, 25.5, 25.5);
                xbox->add();
                box.push_back(xbox);
            }

            bulletPosAndRotBone();

            body = model.getCurrentAnimatedMesh(0);
            body.clearColors();

            ofEnableBlendMode(OF_BLENDMODE_ALPHA);
            glEnable(GL_DEPTH_TEST);

            ofEnableLighting();
            glShadeModel(GL_SMOOTH);
            light.enable();
            light.setSpotlight();
            ofEnableSeparateSpecularLight();

            wireframe = false;
        }

        //--------------------------------------------------------------
        void update(){
            ofSetWindowTitle(ofToString(ofGetFrameRate()));

            world.update();

            bulletPosAndRotBone();
            model.calculateSkeleton();
            body = model.getCurrentAnimatedMesh(0);
        }

        //--------------------------------------------------------------
        void draw(){
            ofBackgroundGradient(200,0);

            cam.begin();
            world.drawDebug();

            ofTranslate(box[0]->getPosition().x,box[0]->getPosition().y-200,box[0]->getPosition().z);
            ofRotateX(box[0]->getRotationAxis().x);
            ofRotateZ(box[0]->getRotationAxis().z);
            ofRotateY(box[0]->getRotationAxis().y);

            ofScale(model.getNormalizedScale()-120, model.getNormalizedScale()-120, model.getNormalizedScale()-120);

            ofPushMatrix();
            body.drawWireframe();
            ofSetColor(ofColor::white);
            for(int i=0;i<(int)model.skeleton.size();i++){
                //model.skeleton[i].mesh.drawWireframe();
                ofMesh face1;
                face1.addVertices(&model.skeleton[i].mesh.getVertices()[0],4);
                ofPushStyle();
                ofSetColor(ofColor::red);
                ofDrawSphere(face1.getCentroid(),0.03);
                ofPopStyle();
                box[i]->getRigidBody()->translate(btVector3(face1.getCentroid().x,face1.getCentroid().y,face1.getCentroid().z));
            }
            ofPopMatrix();

            cam.end();

            ofVec3f gravity = world.getGravity();
            stringstream ss;
            ss << "Gravity(up/down/left/right): x=" << gravity.x << " y= " << gravity.y << " z= " << gravity.z << endl;
            ofSetColor(255,255,255);
            ofDrawBitmapString(ss.str().c_str(), 20, 20);
        }

        //--------------------------------------------------------------
        void keyPressed(int key){
            ofVec3f gravity = world.getGravity();
            if(key=='w'){
                wireframe = !wireframe;
            }

            if(key == 'g'){
                for(int i = 0; i < model.skeleton.size(); i++ ){
                    delete box[i];
                }
                box.clear();
                for(int i = 0; i < model.skeleton.size(); i++ ){
                    ofMesh face1;
                    face1.addVertices(&model.skeleton[i].mesh.getVertices()[0],4);
                    ofxBulletBox* xbox = new ofxBulletBox();
                    xbox->create(world.world, ofVec3f(face1.getCentroid().x,face1.getCentroid().y-400,face1.getCentroid().z-200), 1.0, 25.5, 25.5, 25.5);
                    xbox->add();
                    box.push_back(xbox);
                }
            }

            switch(key){
                case OF_KEY_UP:
                    gravity.y -= 5.;
                    world.setGravity( gravity );
                    break;
                case OF_KEY_DOWN:
                    gravity.y += 5.;
                    world.setGravity( gravity );
                    break;
                case OF_KEY_RIGHT:
                    gravity.x += 5.;
                    world.setGravity( gravity );
                    break;
                case OF_KEY_LEFT:
                    gravity.x -= 5.;
                    world.setGravity( gravity );
                    break;
            }
        }
};

#!/usr/bin/env python
# ---------------------------------------------------------------------------------------------
# Copyright (c) 2012-2015, Johannes Hezer for Studio Rakete (j.hezer@studiorakete.de)

'''
Todos:
make placement relative to the Read node -- done 
detect Vray vs PrMan compliant matrices + Y/Z switch for Vray
'''

import nuke
import os
import math
import logging

logger = logging.getLogger('ExrToCamera')

def getMetadataMatrix(meta_list):
    m = nuke.math.Matrix4()
    try:
        for i in range (0,16) :
            m[i] = meta_list[i]   
    except:
        m.makeIdentity()
    return m    

def ExrToCamera(node=False, gui=True):
    if not node:
        try:
            node = nuke.selectedNode()
        except:
            nuke.message('select a read node with exrs')
            return
    if node.Class() != 'Read':
        nuke.message('no read node selected')
        return
    fileName = node['file'].getValue()
    if not fileName.endswith('.exr'):
        nuke.message('no exr found cant proceed')
        return
    if fileName.__contains__('_%v.'):
        maxViews = 2
    else:
        maxViews = 1
        
    mDat = node.metadata()
    reqFields = ['exr/%s' % i for i in ('worldToCamera', 'worldToNDC')]
    addFields = ['exr/%s' % i for i in ('SR_focalLength','SR_interAxial','SR_zeroParallax')]
    if not set( reqFields ).issubset( mDat ):
        nuke.message('no basic matrices for camera found')
        logger.info('no basic matrices for camera found')
        return
    else:
        logger.info('found needed data')
    
    if set (addFields).issubset(mDat):
        customMeta = True
        logger.info('Custom metadata found')
    else:
        customMeta = False
        logger.info('No Custom Metadata found! Cant produce correct stereo Cams')
        
    hAperture = 36.0
    vAperture = hAperture/1.85

    first = node.firstFrame()
    last = node.lastFrame()
    if gui:
        ret = nuke.getFramesAndViews( 'Create Camera from Metadata', '%s-%s' %( first, last ),maxviews = maxViews  )
        if not ret:
            return
        fRange = nuke.FrameRange( ret[0] )
        camViews = (ret[1])
    else:
        '''
        this only comes of use if it is launched with gui=False
        '''
        try:
            import pyseq
        except:
            logger.info('no pyseq found cannot proceed')
            return
        else:
            seq = pyseq.img2pyseq(node['file'].evaluate())
            if seq.views:
                camViews = ['left','right']
            else:
                camViews = ['left']
            fRange = seq.frames()
        
    if len(camViews) > 1:
        stereo = True
    else:
        stereo = False
    if stereo:
        card = nuke.nodes.Card(inputs=[None],xpos = node.xpos()-400, ypos = node.ypos()-150)
        card['useMatrix'].setValue( True )
        card['deform_rows'].setValue(1)
        card['deform_columns'].setValue(1)
        card['render_mode'].setValue('off')
        card['display'].setValue('wireframe')
        card['name'].setValue('zeroP')
        scene = nuke.nodes.Scene(inputs=[card], xpos = node.xpos()-375, ypos = node.ypos()-100)
        join = nuke.nodes.JoinViews(xpos = node.xpos()-200, ypos = node.ypos()-50) 
    else:
        scene = nuke.nodes.Scene(xpos = node.xpos()-375, ypos = node.ypos()-100)
    const = nuke.nodes.Constant(xpos = node.xpos()-400, ypos = node.ypos()+25)

    for e,act in enumerate(camViews):
        cam = nuke.nodes.Camera (name="Camera %s" % act, xpos = node.xpos()-250+(100*e), ypos = node.ypos()-150)
        cam['useMatrix'].setValue( True )
        cam['haperture'].setValue( hAperture )
        cam['vaperture'].setValue( vAperture )

        if stereo:
            join.setInput(e,cam)
            card['z'].setAnimated()
            card['matrix'].setAnimated()
            for k in ( 'focal', 'matrix', 'win_translate'):
                cam[k].setAnimated()
        else:
            for k in ( 'focal', 'matrix'):
                cam[k].setAnimated()
        
        task = nuke.ProgressTask('Baking camera from meta data in %s' % node.name() )

        for curTask, frame in enumerate( fRange ):
            if task.isCancelled():
                break
            task.setMessage( 'processing frame %s' % frame )

            wTC = node.metadata('exr/worldToCamera',frame, act)
            wTN = node.metadata('exr/worldToNDC')

        # do the matrix math for rotation and translation
        
            matrixList = wTC
            camMatrix = getMetadataMatrix(matrixList)
            
            ### get the focal lenght out of the matrices
            invCamMatrix  = camMatrix.inverse()
            worldProjectionMatrix = getMetadataMatrix(wTN)
            projectionMatrix = worldProjectionMatrix * invCamMatrix
            vectorX = nuke.math.Vector4(1,0,1,1)
            vectorXProjected = projectionMatrix.transform(vectorX)
            pxProjected = vectorXProjected.x / vectorXProjected.w
            focal = (pxProjected * hAperture) / 2.0
            
            cam['focal'].setValueAt(  float( focal ), frame )
            
            if customMeta and stereo:    
                focal = float ( node.metadata('exr/SR_focalLength', frame, act))
                iA = float (node.metadata('exr/SR_interAxial',frame,act))
                zeroP = float( node.metadata('exr/SR_zeroParallax',frame,act))
                
                delta1 = (0.5*iA*(1.0/10.0*focal))
                delta2 =(2.54*zeroP)
                delta = delta1/delta2
                lensShift = ((2/hAperture)*delta*25.4)*-1
    
                if act == 'left':
                    lensShift = 0.0
                    card['z'].setValueAt(  float( zeroP ), frame ) 
                else:
                    lensShift = float(lensShift)
                    
                cam['win_translate'].setValue( lensShift, 0 , frame )
                cam['focal'].setValueAt(  float( focal ), frame )
            
            flipZ=nuke.math.Matrix4()
            flipZ.makeIdentity()
            flipZ.scale(1,1,-1)
         
            transposedMatrix = nuke.math.Matrix4(camMatrix)
            transposedMatrix.transpose()
            transposedMatrix=transposedMatrix*flipZ
            invMatrix=transposedMatrix.inverse()
        
            for i in range(0,16):
                matrixList[i]=invMatrix[i]
            
            for i, v in enumerate( matrixList ):
                cam[ 'matrix' ].setValueAt( v, frame, i)
            if stereo:
                for i, v in enumerate( matrixList ):
                    card[ 'matrix' ].setValueAt( v, frame, i) 
        # UPDATE PROGRESS BAR
            if gui:
                task.setProgress( int( float(curTask) / fRange.frames() *100) )
            else:
                task.setProgress( int( float(curTask) / len(fRange) *100) )
                
    if stereo:
        cam = join
    render = nuke.nodes.ScanlineRender(inputs=[const,scene,cam],name="Render ExrToCamera", xpos = node.xpos()-200, ypos = node.ypos()+50)
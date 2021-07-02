var JoyStick = (function(container,parameters){
    parameters = parameters || {};
    var title = (undefined === parameters.title ? 'joystick' : parameters.title),
        width = (undefined === parameters.width ? 0 : parameters.width),
        height = (undefined === parameters.height ? 0 : parameters.height),
        internalFillColor = (undefined === parameters.internalFillColor ? '#00AA00' : parameters.internalFillColor),
		internalLineWidth = (undefined === parameters.internalLineWidth ? 2 : parameters.internalLineWidth),
		internalStrokeColor = (undefined === parameters.internalStrokeColor ? '#003300' : parameters.internalStrokeColor),
		externalLineWidth = (undefined === parameters.externalLineWidth ? 2 : parameters.externalLineWidth),
		externalStrokeColor = (undefined === parameters.externalStrokeColor ? '#008000' : parameters.externalStrokeColor),
        autoReturnToCenter = (undefined === parameters.autoReturnToCenter ? true : parameters.autoReturnToCenter);
    
    //Create canvas element and add it in the container object
    var objContainer = document.getElementById(container);
    var canvas = document.createElement('canvas');
        canvas.id = title;
    if (width == 0) width = objContainer.clientWidth;
    if (height == 0) height = objContainer.clientHeight;
        canvas.width = width;
        canvas.height = height;
        objContainer.appendChild(canvas);
    var context = canvas.getContext('2d');

    var pressed = 0;
    var circumference = 2*Math.PI;
    var internalRadius = (canvas.width-((canvas.width/2)+10))/2;
    var maxMoveStick = internalRadius + 5;
    var externalRadius = internalRadius + 30;
    var centerX = canvas.width/2;
    var centerY = canvas.height/2;
    var directionHorizontalLimitPos = canvas.width/10;
    var directionHorizontalLimitNeg = directionHorizontalLimitPos * -1;
    var directionVerticalLimitPos = canvas.height /10;
    var directionVerticalLimitNeg = directionVerticalLimitPos * -1;
    var movedX = centerX;
    var movedY = centerY;

    // Memeriksa apakah device mampu disentuh atau tidak
    if("ontouchstart" in document.documentElement)
    {
        canvas.addEventListener('touchstart', onTouchStart, false);
        canvas.addEventListener('touchmove', onTouchMove, false);
        canvas.addEventListener('touchend', onTouchEnd, false);
    }
    else
    {
		canvas.addEventListener('mousedown', onMouseDown, false);
		canvas.addEventListener('mousemove', onMouseMove, false);
		canvas.addEventListener('mouseup', onMouseUp, false);        
    }

    // Menggambar objek
    drawExternal();
    drawInternal(centerX,centerY);
    // ****************
    // Private Methods
    //*****************

    // Menggambar lingkaran luar sbg posisi referensi
    function drawExternal()
    {
        context.beginPath();
        context.arc(centerX,centerY, externalRadius, 0, circumference, false);
        context.lineWidth = externalLineWidth;
        context.strokeStyle = externalStrokeColor;
        context.stroke();
    }
    //Menggambar analog
    function drawInternal()
    {
        context.beginPath();
        if(movedX < internalRadius) movedX = maxMoveStick;
        if((movedX+internalRadius)>canvas.width) movedX = canvas.width-(maxMoveStick);
        if(movedY < internalRadius) movedY = maxMoveStick;
        if((movedY+internalRadius)>canvas.height) movedY = canvas.height -(maxMoveStick);
        context.arc(movedX, movedY, internalRadius, 0, circumference, false);
        //radial gradient
        var grd = context.createRadialGradient(centerX, centerY, 5, centerX, centerY, 200);
        //warna terang
        grd.addColorStop(0, internalFillColor);
        //warna gelap
        grd.addColorStop(1, internalStrokeColor);
        context.fillStyle = grd;
        context.fill();
        context.lineWidth = internalLineWidth;
        context.strokeStyle = internalStrokeColor;
        context.stroke();
    }

    //------------------------
    //Event untuk layar sentuh
    //------------------------
    function onTouchStart(event)
    {
        pressed = 1;
    }
    function onTouchMove(event)
    {
        event.preventDefault(); // menghindari browser melakukan scroll/zoom
        if(pressed == 1 && event.targetTouches[0].target == canvas)
        {
            movedX = event.targetTouches[0].pageX;
            movedY = event.targetTouches[0].pageY;
            //offset
            movedX -= canvas.offsetLeft;
            movedY -= canvas.offsetTop;
            //delete canvas
            context.clearRect(0,0,canvas.width, canvas.height);
            //gambar ulang objek
            drawExternal();
            drawInternal();
        }
    }
    function onTouchEnd(event)
    {
        pressed= 0;
        // reset posisi
        if(autoReturnToCenter)
        {
            movedX = centerX;
            movedY = centerY;
        }
        //delete canvas
        context.clearRect(0,0,canvas.width,canvas.height);
        //gambar ulang objek
        drawExternal();
        drawInternal();
    }

    //-----------------
    //Event untuk mouse
    //-----------------
    function onMouseDown(event)
    {
        pressed = 1;
    }
    function onMouseMove(event)
    {
        if(pressed == 1)
        {
            movedX=event.pageX;
            movedY=event.pageY;
            //offset
            movedX -= canvas.offsetLeft;
            movedY -= canvas.offsetTop;
            //delete canvas
            context.clearRect(0,0,canvas.width,canvas.height);
            //gambar ulang objek
            drawExternal();
            drawInternal();
        }
    }
    function onMouseUp(event)
    {
        pressed = 0;
        if(autoReturnToCenter)
        {
            movedX = centerX;
            movedY = centerY;
        }
        //delete canvas
        context.clearRect(0,0,canvas.width,canvas.height);
        //gambar ulang objek
        drawExternal();
        drawInternal();
    }


    // ****************
    // Public Methods
    //*****************
    
    // Width dari canvas, akan me return pixel dari width
    this.GetWidth = function()
    {
        return canvas.width;
    };
    // Height dari canvas
    this.GetHeight = function()
    {
        return canvas.height;
    };

    //posisi relatif X
    this.GetPosX = function()
    {
        return movedX;
    };
    //posisi relatif Y
    this.GetPosY = function()
    {
        return movedY;
    };

    //Nilai X
    this.GetX = function()
    {
        return (100*((movedX - centerX)/maxMoveStick)).toFixed();
    };
    //Nilai Y
    this.GetY = function()
    {
        return ((100*((movedY - centerY)/maxMoveStick))*-1).toFixed();
    };
});
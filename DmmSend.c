/**
	@file
	dmmsend - a max object shell
	jeremy bernstein - jeremy@bootsquad.com	

	@ingroup	examples	
*/

#include "ext.h"							// standard Max include, always required
#include "ext_obex.h"						// required for new style Max object

#include "DmmDriver.h"

#define MAX_ACCEL 4
#define MAX_SPEED 1

void SerialWrite(char c, void* hook);
void ReportPosition(long value, void* hook);

////////////////////////// object struct
typedef struct _dmmsend 
{
	t_object    ob;			// the object itself (must be first)
    void *m_serialOutlet;
    void *m_posOutlet;
    DmmProtocolState_t state;
    long pos_cache;
    long speed_cache;
} t_dmmsend;

///////////////////////// function prototypes
//// standard set
void *dmmsend_new(t_symbol *s, long argc, t_atom *argv);
void dmmsend_free(t_dmmsend *x);
void dmmsend_assist(t_dmmsend *x, void *b, long m, long a, char *s);

//////////////////////// global class pointer variable
void *dmmsend_class;


// Message handlers

void dmmsend_resetOrigin(t_dmmsend *x)
{
    assert(x);
    ResetOrgin(&(x->state), 0);
    post("Zero/Origin Reset to Current Position\n");
}

/*
void dmmsend_intPos(t_dmmsend *x, long pos)
{
    if (pos != x->pos_cache) {
        SetMaxSpeed(&(x->state), 0, MAX_SPEED);
        SetMaxAccel(&(x->state), 0, MAX_ACCEL);
        MoveMotorToAbsolutePosition32(&(x->state), 0, pos);
        post("Move to Position: %d\n",pos);
        x->pos_cache = pos;
    }
}
*/

void dmmsend_speed(t_dmmsend *x, long speed) {
    speed = MAX(-100,MIN(100,speed)); // SAFE MAX SPEEDS
    if (speed != x->speed_cache) {
        SetMaxSpeed(&(x->state), 0, MAX_SPEED);
        SetMaxAccel(&(x->state), 0, MAX_ACCEL);
        MoveMotorConstantRotation(&(x->state),0,speed);
//        post("Move at Contant Speed: %d\n",speed);
        x->speed_cache = speed;
    }
}

void dmmsend_readPos(t_dmmsend *x) {
    ReadMotorPosition32(&(x->state), 0);
}

void dmmsend_intSerial(t_dmmsend *x, long serialByte) {
    if (serialByte & ~0xff) {
        post("Serial Byte out of range, ignored\n");
        return;
    }
    unsigned char byte = (unsigned char)serialByte;
    ReadPackage(&x->state, byte);
}

void SerialWrite(char c, void* hook) {
    t_dmmsend* x = (t_dmmsend*)hook;
    assert(x);
    outlet_int(x->m_serialOutlet, (int)c);
}


void ReportPosition(long pos, void* hook) {
    t_dmmsend* x = (t_dmmsend*)hook;
    assert(x);
    outlet_int(x->m_posOutlet, pos);
}


int C74_EXPORT main(void)
{	
	// object initialization, OLD STYLE
	// setup((t_messlist **)&dmmsend_class, (method)dmmsend_new, (method)dmmsend_free, (short)sizeof(t_dmmsend), 
	//		0L, A_GIMME, 0);
    // addmess((method)dmmsend_assist,			"assist",		A_CANT, 0);  
	
	// object initialization, NEW STYLE
	t_class *c;
	
	c = class_new("dmmsend", (method)dmmsend_new, (method)dmmsend_free, (long)sizeof(t_dmmsend), 
				  0L /* leave NULL!! */, A_GIMME, 0);
	
	/* you CAN'T call this from the patcher */
    class_addmethod(c, (method)dmmsend_assist, "assist", A_CANT, 0);
    //class_addmethod(c, (method)dmmsend_intPos, "position", A_LONG, 0);
    class_addmethod(c, (method)dmmsend_speed, "speed", A_LONG, 0);
    class_addmethod(c, (method)dmmsend_resetOrigin,"resetOrigin", 0);
    class_addmethod(c, (method)dmmsend_readPos,"readPosition", 0);
    class_addmethod(c, (method)dmmsend_intSerial, "serialByte", A_LONG, 0);

	
	class_register(CLASS_BOX, c); /* CLASS_NOBOX */
	dmmsend_class = c;

	return 0;
}

void dmmsend_assist(t_dmmsend *x, void *b, long m, long number, char *s)
{
	if (m == ASSIST_INLET) { // inlet
		sprintf(s, "Command Input & Serial Feedback");
	} 
	else {	// outlet
		//sprintf(s, "Serial Bytes, connect to a Serial Object, %ld",number);
	}
}

void dmmsend_free(t_dmmsend *x)
{
	;
}

/*
 A_GIMME signature =
	t_symbol	*s		objectname
	long		argc	num additonal args
	t_atom		*argv	array of t_atom structs
		 type = argv->a_type
		 if (type == A_LONG) ;
		 else if (type == A_FLOAT) ;
		 else if (type == A_SYM) ;
*/
/*
	t_symbol {
		char *s_name;
		t_object *s_thing;
	}
*/
void *dmmsend_new(t_symbol *s, long argc, t_atom *argv)
{
    t_dmmsend *x = (t_dmmsend *)object_alloc(dmmsend_class); // object instantiation, NEW STYLE
    
	if (x != NULL) {
        //object_post((t_object *)x, "a new %s object was instantiated: %p", s->s_name, x);
        //object_post((t_object *)x, "it has %ld arguments", argc);
        
        /*
        for (long i = 0; i < argc; i++) {
            if ((argv + i)->a_type == A_LONG) {
                object_post((t_object *)x, "arg %ld: long (%ld)", i, atom_getlong(argv+i));
            } else if ((argv + i)->a_type == A_FLOAT) {
                object_post((t_object *)x, "arg %ld: float (%f)", i, atom_getfloat(argv+i));
            } else if ((argv + i)->a_type == A_SYM) {
                object_post((t_object *)x, "arg %ld: symbol (%s)", i, atom_getsym(argv+i)->s_name);
            } else {
                object_error((t_object *)x, "forbidden argument");
            }
        }
        */
        x->pos_cache = LONG_MIN;
        memset(&(x->state),0,sizeof(x->state));
        x->state.SerialWritePtr = &SerialWrite;
        x->state.ReportPositionPtr = &ReportPosition;
        x->state.hook = (void*)x;
        
        post("DmmSend Created at with MaxSpeed:%d, and Max Acceleration: %d\n",MAX_SPEED,MAX_ACCEL);
        
        // add inlet
        // one is make by default
        
        //intin(...) to creat more inletls
        
        // add outlets
        x->m_posOutlet = intout((t_object *)x);
        x->m_serialOutlet = intout((t_object *)x);
        return x;

	}
	return (x);
}



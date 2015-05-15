/**
	@file
	dmmsend - a max object shell
	jeremy bernstein - jeremy@bootsquad.com	

	@ingroup	examples	
*/

#include "ext.h"							// standard Max include, always required
#include "ext_obex.h"						// required for new style Max object

#include "DmmDriver.h"

////////////////////////// object struct
typedef struct _dmmsend 
{
	t_object					ob;			// the object itself (must be first)
    void *m_outlet1;
    DmmProtocolBuffer_t protcolBuffer;
    long pos_cache;
} t_dmmsend;

///////////////////////// function prototypes
//// standard set
void *dmmsend_new(t_symbol *s, long argc, t_atom *argv);
void dmmsend_free(t_dmmsend *x);
void dmmsend_assist(t_dmmsend *x, void *b, long m, long a, char *s);

//////////////////////// global class pointer variable
void *dmmsend_class;


// Message handlers

void dmmsend_bang(t_dmmsend *x)
{
    assert(x);
    ResetOrgin(&(x->protcolBuffer), 0);
    post("Zero/Origin Reset to Current Position\n");
}


void dmmsend_int(t_dmmsend *x, long pos)
{
    if (pos != x->pos_cache) {
        MoveMotorToAbsolutePosition32(&(x->protcolBuffer), 0,pos);
        x->pos_cache = pos;
    }
}


void SerialWrite(char c, void* hook) {
    t_dmmsend* x = (t_dmmsend*)hook;
    assert(x);
    //post("Serial Out %x\n",c);
    outlet_int(x->m_outlet1, (int)c);
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
    class_addmethod(c, (method)dmmsend_assist,			"assist",		A_CANT, 0);
    class_addmethod(c, (method)dmmsend_int, "int", A_LONG, 0);
    class_addmethod(c, (method)dmmsend_bang, "bang", 0);

	
	class_register(CLASS_BOX, c); /* CLASS_NOBOX */
	dmmsend_class = c;

	//post("I am the dmmsend object");
	return 0;
}

void dmmsend_assist(t_dmmsend *x, void *b, long m, long a, char *s)
{
	if (m == ASSIST_INLET) { // inlet
		sprintf(s, "I am inlet %ld", a);
	} 
	else {	// outlet
		sprintf(s, "I am outlet %ld", a); 			
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
        object_post((t_object *)x, "a new %s object was instantiated: %p", s->s_name, x);
        object_post((t_object *)x, "it has %ld arguments", argc);
        
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
        
        
        x->pos_cache = LONG_MIN;
        memset(&(x->protcolBuffer),0,sizeof(x->protcolBuffer));
        x->protcolBuffer.SerialWriteFunctionPtr = &SerialWrite;
        x->protcolBuffer.hook = (void*)x;
        
        // add inlets
        
        // one by default
        
        // add outlets
        x->m_outlet1 = intout((t_object *)x);
        return x;

	}
	return (x);
}



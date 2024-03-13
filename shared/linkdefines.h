#ifndef __LINKDEFINES__
#define __LINKDEFINES__
#pragma once

#define LINKINSERTEND_A(node,first,last)	{ \
												(node)->m_prev=last; \
												(node)->m_next=NULL; \
												if(last) \
													last->m_next=node; \
												else \
													first=node; \
												last=node; \
											}
#define LINKINSERTFIRST_A(node,first,last)	{ \
												(node)->m_next=first; \
												(node)->m_prev=NULL; \
												if(first) \
													first->m_prev=node; \
												else \
													last=node; \
												first=node; \
											}

#define LINKREMOVE_A(node,first,last)			{ \
												if((node)->m_prev) \
													(node)->m_prev->m_next=(node)->m_next; \
												else \
													first=(node)->m_next; \
												if((node)->m_next) \
													(node)->m_next->m_prev=(node)->m_prev; \
												else \
													last=(node)->m_prev; \
												(node)->m_next=NULL; \
												(node)->m_prev=NULL; \
											}



#define LINKINSERTBEFORE_A(node,beforeThis,first,last) \
											{ \
												if(beforeThis) \
												{ \
													(node)->m_prev=(beforeThis)->m_prev; \
													(node)->m_next=beforeThis; \
													if((node)->m_prev) \
														(node)->m_prev->m_next=node; \
													else \
														first=node; \
													(beforeThis)->m_prev=node; \
												} \
												else \
												{ \
													LINKINSERTFIRST_A(node,first,last); \
												} \
											}


#define LINKINSERTEND(node,first,last)			{ \
												(node)->m_pPrev=last; \
												(node)->m_pNext=NULL; \
												if(last) \
													last->m_pNext=node; \
												else \
													first=node; \
												last=node; \
											}

#define LINKINSERTFIRST(node,first,last)	{ \
												(node)->m_pNext=first; \
												(node)->m_pPrev=NULL; \
												if(first) \
													first->m_pPrev=node; \
												else \
													last=node; \
												first=node; \
											}

#define LINKINSERTBEFORE(node,beforeThis,first,last) \
											{ \
												if(beforeThis) \
												{ \
													(node)->m_pPrev=(beforeThis)->m_pPrev; \
													(node)->m_pNext=beforeThis; \
													if((node)->m_pPrev) \
														(node)->m_pPrev->m_pNext=node; \
													else \
														first=node; \
													(beforeThis)->m_pPrev=node; \
												} \
												else \
												{ \
													LINKINSERTFIRST(node,first,last); \
												} \
											}

#define LINKINSERTAFTER(node,AfterThis,first,last) \
											{ \
												if(AfterThis) \
												{ \
													(node)->m_pNext=AfterThis->m_pNext; \
													(node)->m_pPrev=AfterThis; \
													if((node)->m_pNext) \
														(node)->m_pNext->m_pPrev = node; \
													else \
														last = node; \
													AfterThis->m_pNext = node; \
												} \
												else \
												{ \
													LINKLISTINSERTEND(node,first,last) \
												} \
											}

#define LINKREMOVE(node,first,last)			{ \
												if((node)->m_pPrev) \
													(node)->m_pPrev->m_pNext=(node)->m_pNext; \
												else \
													first=(node)->m_pNext; \
												if((node)->m_pNext) \
													(node)->m_pNext->m_pPrev=(node)->m_pPrev; \
												else \
													last=(node)->m_pPrev; \
												(node)->m_pNext=NULL; \
												(node)->m_pPrev=NULL; \
											}

#define VALIDATEINLIST(classname,node,first,last){ \
												classname* p=first; \
												while(p) { \
													if(p==node)break; \
													p=p->m_pNext; \
												} \
												if(!p)INT3; \
											}


//Custom defines used when m_pPrev and m_pNext has different names

#define LINKINSERTENDCUSTOM(prevname,nextname,node,first,last){ \
												(node)->prevname=last; \
												(node)->nextname=NULL; \
												if(last) \
													last->nextname=node; \
												else \
													first=node; \
												last=node; \
											}

#define LINKREMOVECUSTOM(prevname,nextname,node,first,last){ \
												if((node)->prevname) \
													(node)->prevname->nextname=(node)->nextname; \
												else \
													first=(node)->nextname; \
												if((node)->nextname) \
													(node)->nextname->prevname=(node)->prevname; \
												else \
													last=(node)->prevname; \
												(node)->nextname=NULL; \
												(node)->prevname=NULL; \
											}

#define VALIDATEINLISTCUSTOM(classname,prevname,nextname,node,first,last){ \
												classname* p=first; \
												while(p) { \
													if(p==node)break; \
													p=p->nextname; \
												} \
												if(!p)INT3; \
											}

#endif//__LINKDEFINES__
